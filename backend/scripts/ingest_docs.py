import os
import re
import yaml
import asyncio
import uuid
from pathlib import Path
from typing import List, Dict, Any
from qdrant_client import models

from app.services.embeddings import EmbeddingService
from app.services.qdrant_client import QdrantClient
from app.config import settings

def read_mdx_file(file_path: Path) -> str:
    with open(file_path, 'r', encoding='utf-8') as f:
        return f.read()

def parse_frontmatter_and_content(content: str) -> tuple[Dict, str]:
    frontmatter = {}
    body = content
    if content.startswith('---'):
        parts = content.split('---', 2)
        if len(parts) > 2:
            try:
                frontmatter = yaml.safe_load(parts[1])
                body = parts[2].strip()
            except yaml.YAMLError as e:
                print(f"Warning: Could not parse frontmatter: {e}")
    return frontmatter, body

def semantic_chunking(content: str, source_file: str, frontmatter: Dict) -> List[Dict]:
    chunks = []
    current_chunk_content = ""
    current_header = ""
    in_code_block = False

    lines = content.split('\n')
    for line in lines:
        if line.strip().startswith('```'):
            in_code_block = not in_code_block
            current_chunk_content += line + '\n'
            continue

        if in_code_block:
            current_chunk_content += line + '\n'
        elif re.match(r'^#+\s', line):  # Markdown header
            if current_chunk_content.strip():
                chunks.append({
                    "content": current_chunk_content.strip(),
                    "header": current_header,
                    "source_file": source_file,
                    "frontmatter": frontmatter
                })
            current_header = line.strip()
            current_chunk_content = line + '\n'
        else:
            current_chunk_content += line + '\n'

    if current_chunk_content.strip():
        chunks.append({
            "content": current_chunk_content.strip(),
            "header": current_header,
            "source_file": source_file,
            "frontmatter": frontmatter
        })

    return chunks

async def main():
    # Initialize services
    embedding_service = EmbeddingService()
    qdrant_client = QdrantClient()

    # Recreate Qdrant collection for fresh ingestion
    await qdrant_client.recreate_collection(vector_size=embedding_service.embedding_dim)

    # Directory containing MDX files
    docs_base_path = Path('../frontend/docs')

    mdx_files = list(docs_base_path.rglob('*.mdx'))

    if not mdx_files:
        print(f"No MDX files found in {docs_base_path}")
        return

    print(f"Found {len(mdx_files)} MDX files in {docs_base_path}")

    for file_index, mdx_file_path in enumerate(mdx_files):
        print(f"\n--- Processing file {file_index + 1}/{len(mdx_files)}: {mdx_file_path} ---")
        try:
            content = read_mdx_file(mdx_file_path)
            frontmatter, body = parse_frontmatter_and_content(content)
            chunks_data = semantic_chunking(body, str(mdx_file_path), frontmatter)

            print(f"  Extracted {len(chunks_data)} chunks.")

            points_to_upsert = []
            for i, chunk in enumerate(chunks_data):
                try:
                    embedding = await embedding_service.generate_embedding(chunk['content'])
                    points_to_upsert.append(models.PointStruct(
                        id=str(uuid.uuid4()), # unique ID
                        vector=embedding,
                        payload=chunk
                    ))
                except Exception as e:
                    print(f"    Error generating embedding for chunk {i+1} in {mdx_file_path.name}: {e}")

            if points_to_upsert:
                await qdrant_client.upsert_points(points=points_to_upsert)
                print(f"  Successfully upserted {len(points_to_upsert)} points from {mdx_file_path.name}")
            else:
                print(f"  No points to upsert for {mdx_file_path.name}")

        except Exception as e:
            print(f"Error processing file {mdx_file_path}: {e}")

    print("\n--- Ingestion process complete ---")

if __name__ == "__main__":
    asyncio.run(main())