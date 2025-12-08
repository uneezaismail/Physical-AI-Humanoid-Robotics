"""
Agent definition using openai-agents SDK.
"""

from agents import Agent, Runner, function_tool, RunContextWrapper
from pydantic import BaseModel
from typing import List, Dict, Optional, Any
import logging
from dataclasses import dataclass, field

from .services.qdrant_client import qdrant_client
from .services.embeddings import embedding_service

logger = logging.getLogger(__name__)

@dataclass
class AgentContext:
    """Context to store state during agent execution."""
    retrieved_chunks: List[Dict] = field(default_factory=list)

@function_tool
async def search_textbook(ctx: RunContextWrapper[AgentContext], query: str) -> str:
    """
    Search the textbook/course content for a given query.
    Use this tool when the user asks a question about the book, robotics, ROS, or the course.
    """
    logger.info(f"Agent tool 'search_textbook' called with query: {query}")

    # Access the actual context object
    context_data = ctx.context

    # Generate embedding
    embedding = await embedding_service.generate_embedding(query)

    # Search Qdrant
    results = await qdrant_client.search_points(embedding, limit=5)

    if not results:
        return "No relevant information found in the textbook."

    # Store results in context for the API response
    context_data.retrieved_chunks.extend(results)

    # Format for the LLM
    formatted_results = []
    for i, res in enumerate(results, 1):
        payload = res.get("payload", {})
        content = payload.get("content", "")
        
        # Extract metadata with fallbacks
        frontmatter = payload.get("frontmatter", {})
        title = frontmatter.get("title", payload.get("title", "Unknown"))
        section = payload.get("header", payload.get("section", ""))
        source_file = payload.get("source_file", "Unknown")

        # Extract more user-friendly path for citation
        # Convert source_file path like "../frontend/docs/part-1-foundations-lab/chapter-01-embodied-ai.mdx"
        # to a frontend-friendly path like "part-1-foundations-lab/chapter-01-embodied-ai"
        if source_file and source_file != "Unknown":
            # Remove the "../frontend/docs/" prefix and ".mdx" suffix
            clean_path = source_file.replace("../frontend/docs/", "").replace(".mdx", "")
        else:
            clean_path = "Unknown"

        formatted_results.append(f"Source {i}: [{title} - {section}] - Path: {clean_path}\n{content}\n")

    return "\n---\n".join(formatted_results)

# Define the agent with typed context
textbook_agent = Agent[AgentContext](
    name="Physical AI Tutor",
    instructions="""You are an AI teaching assistant for a "Physical AI & Humanoid Robotics" textbook. You are an expert in robotics, AI, and related fields.

    Your responsibilities:
    1. Answer student questions using the 'search_textbook' tool. First, ALWAYS call the search_textbook tool with the user's query.
    2. If the user greets you (e.g., "hi", "hello"), greet back warmly and ask "How can I help you with the course material today?".
    3. If the user provides selected text or specific context, explain it.
    4. ALWAYS cite your sources if you use information from the textbook (e.g., "According to Chapter 1...").
    5. If the tool returns no results for the exact term, but the topic is related to robotics, AI, or the course material, make educated inferences based on the available information and explain what you can from the textbook content. For example, if asked about "URDF" and the tool finds information about "robot description" or "modeling", you should connect these concepts.
    6. When citing sources, provide the specific path where students can find the information in the book (e.g., "You can find more about this in Part 1, Chapter 3").
    7. Be comprehensive, encouraging, and educational. Explain concepts clearly and connect related topics.
    8. If the user asks about something that is clearly outside the scope of the textbook and no relevant information is found, then admit you don't have that specific information.

    Do not make up information not present in the tool output, but do connect related concepts when possible.
    """,
    tools=[search_textbook],
)
