import asyncio
import os
import sys

# Add the current directory to sys.path to ensure 'app' can be imported
sys.path.append(os.getcwd())

from app.agent import textbook_agent, AgentContext
from agents import Runner, RunConfig, OpenAIChatCompletionsModel
from openai import AsyncOpenAI
from app.config import settings

async def main():
    print("Testing Agent Response...")
    
    # Ensure API keys are present
    if not settings.gemini_api_key:
        print("Error: GEMINI_API_KEY is missing from settings.")
        return
    
    # Query related to the textbook content
    query = "explain chapter three of this book"
    print(f"Query: {query}")

    try:
        # Configure Gemini Client
        client = AsyncOpenAI(
            api_key=settings.gemini_api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        )

        model = OpenAIChatCompletionsModel(
            model="gemini-2.5-flash",
            openai_client=client,
        )

        run_config = RunConfig(model=model)
        agent_context = AgentContext()

        print("Running agent...")
        result = await Runner.run(
            textbook_agent,
            input=query,
            context=agent_context,
            run_config=run_config,
        )

        print("\n--- Agent Response ---")
        print(result.final_output)
        print("\n--- Retrieved Chunks (RAG Verification) ---")
        # We need to inspect the formatted output from the tool call to see how the agent sees it, 
        # BUT for this script we can check how the AGENT sees the data (which is formatted inside the tool)
        # OR we can inspect the raw chunks in agent_context.retrieved_chunks.
        
        # Let's simulate what the API does to verify the metadata extraction:
        if agent_context.retrieved_chunks:
            for i, chunk in enumerate(agent_context.retrieved_chunks, 1):
                payload = chunk.get("payload", {})
                
                # SIMULATING THE FIXED LOGIC FROM api/chat.py
                frontmatter = payload.get("frontmatter", {})
                title = frontmatter.get("title", payload.get("title", "Unknown"))
                section = payload.get("header", payload.get("section", "Unknown"))
                
                print(f"{i}. {title} - {section}")
        else:
            print("No chunks retrieved.")

    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(main())