import pytest
from fastapi.testclient import TestClient
import sys
from pathlib import Path

# Add the backend directory to sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from app.main import app

client = TestClient(app)

def test_read_docs():
    """
    Test that the Swagger UI documentation endpoint (/docs) is accessible.
    """
    response = client.get("/docs")
    assert response.status_code == 200
    assert "swagger-ui" in response.text.lower()

def test_read_openapi_json():
    """
    Test that the OpenAPI JSON schema endpoint (/openapi.json) is accessible.
    """
    response = client.get("/openapi.json")
    assert response.status_code == 200
    assert response.json()["info"]["title"] == "Physical AI & Humanoid Robotics Textbook API"

def test_read_root():
    """
    Test that the root endpoint (/) returns the API information.
    """
    response = client.get("/")
    assert response.status_code == 200
    assert response.json() == {
        "message": "Physical AI & Humanoid Robotics Textbook API",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/api/health"
    }
