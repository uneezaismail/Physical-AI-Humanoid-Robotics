import json
import logging
from typing import Any, AsyncGenerator, Dict

from agents import Agent, OpenAIChatCompletionsModel, Runner, function_tool
from openai import AsyncOpenAI, OpenAI

from app.config import settings
from app.services.qdrant_client import qdrant_client

logger = logging.getLogger(__name__)


@function_tool
async def search_textbook(query: str, filter_unit: str = None) -> str:
    """
    Retrieves technical content from the Physical AI textbook.

    Args:
        query: The semantic search string (e.g., "Inverse kinematics formula").
        filter_unit: Optional. Limit search to a specific unit (e.g., "01-part-1-foundations-lab").

    Returns:
        JSON string containing list of matches with 'content', 'page_id', and 'relevance_score'.
    """
    try:
        results = await qdrant_client.search_textbook(
            query=query, filter_unit=filter_unit, limit=5
        )
        return json.dumps(results)
    except Exception as e:
        logger.error(f"Error in search_textbook tool: {e}")
        return json.dumps({"error": f"Retrieval failed: {str(e)}"})


class AgentService:
    """Service for orchestrating the RAG chatbot agent."""

    def __init__(self):
        self.openai_client = AsyncOpenAI(
            api_key=settings.gemini_api_key, base_url=settings.gemini_base_url
        )
        self.agent_model = OpenAIChatCompletionsModel(
            model="gemini-2.0-flash", openai_client=self.openai_client
        )
        self.textbook_agent = self._initialize_agent()
        logger.info("AgentService initialized with textbook_agent.")

    def _initialize_agent(self) -> Agent:
        system_prompt = """
        You are the **Physical AI & Humanoid Robotics Textbook Teaching Assistant**. You are an expert in robotics, AI, ROS 2, and humanoid engineering.  │
        Your goal is to help students learn by providing comprehensive, easy-to-understand, and engaging explanations based *strictly* on the textbook    │
        content provided in the context.

       **Guidelines for Answering:**                                                                                                                     │
       1.  **Be an Educator:** Don't just summarize. Explain concepts as if you are teaching a student. Use analogies if helpful (but stick to the       │
       technical accuracy of the text).                                                                                                                        │
       2.  **Synthesize Information:** Combine information from multiple retrieved chunks to form a cohesive answer. Do not just list the search         │
       results.                                                                                                                                                │
       3.  **Depth & Clarity:** Provide detailed answers. If the user asks "What is ROS 2?", explain its architecture, purpose, and key components found │
       in the text, not just "It's a framework."                                                                                                               │
       4.  **Formatting:** Use Markdown (headers, bolding, bullet points) to structure your response effectively and make it easy to read.               │
       5.  **Citations:** You MUST support your specific claims by citing the source. At the end of your response, include a section called              │
       "**References**" listing the specific chapter paths (e.g., `'01-part-1-foundations-lab/...'`) you used.                                                 │
       6.  **Unknowns:** If the retrieved context does not contain the answer, strictly state: "I couldn't find specific information about that in the   │
       textbook." Do not use outside knowledge to fill gaps if it contradicts the "book-only" rule, but you can use general robotics knowledge to bridge small │
                   gaps *if* it aligns with the retrieved context.
        7.  **Selected Text Enhancement:** If the user provides `selected_text` and asks a question (especially a general "explain" or "what is this?"),
                   and the `selected_text` itself is brief (e.g., a phrase, title, or sentence), use the `search_textbook` tool with the `selected_text`
                   as the query to retrieve additional relevant content from the textbook *before* formulating your explanation. This helps provide a more comprehensive answer.        Special Handling:
        - **Greetings:** If the user says 'hi', 'hello', or similar, respond with a friendly greeting and prompt them to ask about book content (e.g., "Hello! I'm your Physical AI Textbook Chatbot. How can I help you learn today? Feel free to ask me about any topic in the book, like ROS 2, humanoid kinematics, or simulation!").
        **Handling Off-Topic Queries:**                                                                                                                   │
        - If the user asks about topics clearly outside the scope of Physical AI, Robotics, ROS 2, Simulators, or the textbook content (e.g., "What is a  │
        black hole?", "How to cook pasta?"), politely decline.                                                                                                  │
        - Response: "I apologize, but I am a specialized assistant for the Physical AI & Humanoid Robotics textbook. I cannot answer questions about      │
        [topic]. Is there anything related to robotics or the course I can help you with?"

        Use the `search_textbook` tool for any query that might be related to the book content to retrieve relevant information.
        """

        return Agent(
            name="TextbookChatbot",
            instructions=system_prompt,
            model=self.agent_model,
            tools=[search_textbook],
        )

    async def stream_agent_response(
        self, user_message: str, selected_text: str = None
    ) -> AsyncGenerator[Dict[str, Any], None]:
        """
        Streams agent responses token-by-token using Server-Sent Events pattern.
        """
        # Construct full input with selected text if provided
        full_input = user_message
        if selected_text:
            full_input = f"User Selected Text:\n{selected_text}\n\nUser Question:\n{user_message}\n\nPlease explain the selected text in the context of the question."

        # Special handling for greetings and out-of-context queries BEFORE running the agent
        lower_message = user_message.lower()
        if any(greeting in lower_message for greeting in ["hi", "hello", "hey"]):
            yield {
                "type": "token",
                "content": "Hello! I'm your Physical AI Textbook Chatbot. How can I help you learn today? Feel free to ask me about any topic in the book, like ROS 2, humanoid kinematics, or simulation!",
            }
            return

        # Basic check for out-of-context. This can be improved with a dedicated tool/agent for classification.
        # For now, we'll assume if it's not a greeting and doesn't seem book-related, it's out of context
        # This is a very simplistic check and will need refinement.
        book_keywords = [
            "ros",
            "robotics",
            "ai",
            "physical ai",
            "kinematics",
            "simulation",
            "hardware",
            "chapter",
        ]
        if not any(
            keyword in lower_message for keyword in book_keywords
        ) and not await self._is_query_book_related(user_message):
            yield {
                "type": "token",
                "content": "I apologize, but my knowledge is focused on the Physical AI & Humanoid Robotics textbook. I can't assist with that topic.",
            }
            return

        # Use synchronous run as fallback due to SDK bug with Gemini streaming
        try:
            # Pass full_input which might include selected text
            result = await Runner.run(self.textbook_agent, input=full_input)
            yield {"type": "token", "content": result.final_output}
            yield {"type": "agent_finish", "output": result.final_output}
        except Exception as e:
            logger.error(f"Agent run failed: {e}")
            yield {
                "type": "token",
                "content": "I encountered an error while processing your request.",
            }

    async def _is_query_book_related(self, query: str) -> bool:
        """
        A simple heuristic to check if a query is likely related to the book content.
        This can be enhanced with a dedicated classification model or a more sophisticated prompt.
        """
        # For now, a very basic check. In a real scenario, an LLM call or a small classification model
        # would be used here.
        return True  # Temporarily assume all non-greeting queries are book-related for initial testing


agent_service = AgentService()
