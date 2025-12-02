"""
FastEmbed embeddings service.

Generates embeddings for text chunks using local FastEmbed models (no API required).
"""

import asyncio
import logging
from typing import List

from openai import OpenAI

from app.config import settings

logger = logging.getLogger(__name__)


class EmbeddingService:
    """Service for generating text embeddings using the Gemini API (via OpenAI SDK)."""

    def __init__(self):
        """
        Initialize OpenAI client for Gemini Embeddings.
        Uses text-embedding-004 model with 768 dimensions as specified.
        """
        self.client = OpenAI(
            api_key=settings.gemini_api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
        )
        self.model_name = "text-embedding-004"
        self.embedding_dim = 768  # As specified in CLAUDE.md

        logger.info(
            f"Initialized Gemini Embeddings with model: {self.model_name}, "
            f"dimension: {self.embedding_dim}"
        )

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text using Gemini API.

        Args:
            text: Input text

        Returns:
            Embedding vector (768 dimensions)
        """
        try:
            response = await asyncio.to_thread(
                self.client.embeddings.create,
                model=self.model_name,
                input=text
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Failed to generate embedding: {str(e)}")
            raise

    async def generate_embeddings_batch(
        self,
        texts: List[str],
        batch_size: int = 32  # Gemini API might have its own batching/rate limits
    ) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in batches using Gemini API.

        Args:
            texts: List of input texts
            batch_size: Number of texts to process at once (default: 32)

        Returns:
            List of embedding vectors
        """
        all_embeddings = []
        try:
            # The OpenAI SDK handles internal batching or we can add it here if needed
            # For now, making a single call to leverage potential API-side batching
            # and to simplify the initial implementation.
            response = await asyncio.to_thread(
                self.client.embeddings.create,
                model=self.model_name,
                input=texts
            )
            all_embeddings = [d.embedding for d in response.data]
            logger.info(f"✅ Total embeddings generated: {len(all_embeddings)}")
            return all_embeddings
        except Exception as e:
            logger.error(f"Failed to generate embeddings: {str(e)}")
            raise


# Global instance
embedding_service = EmbeddingService()
