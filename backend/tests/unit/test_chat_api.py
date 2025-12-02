import pytest
import httpx
import asyncio
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch, MagicMock
import json

import sys
from pathlib import Path

# Add the backend directory to sys.path to allow importing app modules
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent)) # Go up three levels to reach the 'backend' directory
from app.main import app
from app.services.agent_service import agent_service
from agents import Agent, Runner, function_tool

# We will mock the external services to ensure tests are fast and isolated
# For Qdrant search, we'll mock the search_textbook tool
# For Gemini generation, we'll mock the Agent's model interaction

# Mock data for search_textbook tool
MOCK_BOOK_CONTENT = [
    {
        "content": "ROS 2 is the Robot Operating System 2. It is a set of software libraries and tools that help you build robot applications.",
        "page_id": "02-part-2-robotic-nervous-system/04-chapter-4-ros2-architecture/03-core-components.mdx",
        "header": "ROS 2 Core Components",
        "relevance_score": 0.95
    },
    {
        "content": "Humanoid kinematics deals with the study of motion of humanoid robots without considering the forces that cause the motion.",
        "page_id": "04-part-4-ai-robot-brain/20-chapter-20-humanoid-kinematics/00-intro.mdx",
        "header": "Humanoid Kinematics Intro",
        "relevance_score": 0.92
    }
]


@pytest.fixture(scope="module")
def test_app():
    client = TestClient(app)
    yield client


@pytest.mark.asyncio
async def test_chat_stream_greeting(test_app):
    """Test that the chat stream endpoint handles greetings correctly."""
    async with httpx.AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post("/api/chat/stream", json={"query": "Hi there!"})

        assert response.status_code == 200

        events = [json.loads(line.replace("data: ", "")) for line in response.text.replace("\r\n", "\n").strip().split("\n\n") if line.strip()]
        assert len(events) == 1
        assert events[0]["type"] == "token"
        assert "Hello! I'm your Physical AI Textbook Chatbot" in events[0]["content"]


@pytest.mark.asyncio
async def test_chat_stream_out_of_context(test_app):
    """Test that the chat stream endpoint handles out-of-context queries gracefully."""
    # Temporarily modify _is_query_book_related to return False for specific test query
    with patch('app.services.agent_service.AgentService._is_query_book_related', new_callable=AsyncMock) as mock_is_related:
        mock_is_related.return_value = False
        
        async with httpx.AsyncClient(app=app, base_url="http://test") as client:
            response = await client.post("/api/chat/stream", json={"query": "Tell me about the weather today."})

            assert response.status_code == 200

            events = [json.loads(line.replace("data: ", "")) for line in response.text.replace("\r\n", "\n").strip().split("\n\n") if line.strip()]
            assert len(events) == 1
            assert events[0]["type"] == "token"
            assert "I apologize, but my knowledge is focused on the Physical AI & Humanoid Robotics textbook." in events[0]["content"]


@pytest.mark.asyncio
@patch('app.services.agent_service.qdrant_client.search_textbook', new_callable=AsyncMock)
@patch('app.services.agent_service.OpenAI') # Mock the OpenAI client for Gemini API calls
async def test_chat_stream_book_query(mock_openai_client, mock_search_textbook, test_app):
    """Test that the chat stream endpoint handles book-related queries correctly."""
    mock_search_textbook.return_value = MOCK_BOOK_CONTENT

    # Mock the result object returned by Runner.run
    mock_result = MagicMock()
    mock_result.final_output = "This is an answer from the book about ROS 2. You can find more at 02-part-2-robotic-nervous-system/04-chapter-4-ros2-architecture/03-core-components.mdx"

    # Patch Runner.run to return our mock result object
    with patch('agents.Runner.run', new_callable=AsyncMock) as mock_run:
        mock_run.return_value = mock_result
        
        async with httpx.AsyncClient(app=app, base_url="http://test") as client:
            response = await client.post("/api/chat/stream", json={"query": "What is ROS 2?"})

            assert response.status_code == 200

            # Check events - we expect one token event with the full answer and one finish event
            events = [json.loads(line.replace("data: ", "")) for line in response.text.replace("\r\n", "\n").strip().split("\n\n") if line.strip()]
            
            assert len(events) >= 1
            token_events = [e for e in events if e["type"] == "token"]
            assert len(token_events) > 0
            assert "This is an answer from the book about ROS 2." in token_events[0]["content"]

