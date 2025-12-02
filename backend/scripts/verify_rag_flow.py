import asyncio
import os
import sys
from pathlib import Path
import logging

# Add the backend directory to sys.path to allow importing app modules
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from app.services.agent_service import agent_service
from app.config import settings

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from agents import Runner

async def main():
    print(f"Environment: {settings.environment}")
    print(f"Qdrant URL: {settings.qdrant_url}")
    print(f"Gemini Base URL: {settings.gemini_base_url}")
    
    query = "What is ROS 2?"
    print(f"\n--- Testing RAG Flow with Query: '{query}' ---\n")

    try:
        print("Running standard query (bypassing stream to avoid SDK bug)...")
        result = await Runner.run(agent_service.textbook_agent, input=query)
        print(f"\nResult:\n{result.final_output}")

        print(f"\n--- Testing RAG Flow with User's Specific Selected Text Query ---")
        specific_selected_text = "Physical Laws Through Embodiment"
        specific_user_query = "explain"
        
        # The agent service constructs this combined input when selected_text is provided.
        # We need to simulate the agent service's logic here for Runner.run
        if specific_selected_text:
            full_input_for_agent_specific = f"User Selected Text:\n{specific_selected_text}\n\nUser Question:\n{specific_user_query}\n\nPlease explain the selected text in the context of the question."
        else:
            full_input_for_agent_specific = specific_user_query

        print(f"User Query: '{specific_user_query}'")
        print(f"Selected Text: '{specific_selected_text}'")
        print("Running specific selected text query (bypassing stream to avoid SDK bug)...")
        result_specific_selected_text = await Runner.run(agent_service.textbook_agent, input=full_input_for_agent_specific)
        print(f"\nResult for Specific Selected Text:\n{result_specific_selected_text.final_output}")
        
        print("\n\n--- Test Complete ---")
        
    except Exception as e:
        print(f"\n\n!!! Error during RAG verification: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(main())
