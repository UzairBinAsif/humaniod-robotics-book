import os
import logging
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance
from dotenv import load_dotenv
from openai import OpenAI # Import OpenAI

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname_s)s - %(message)s')
logger = logging.getLogger(__name__)

# Load environment variables from .env file
load_dotenv()

# Define the collection name as a constant
COLLECTION_NAME = "humanoid-robotics-book"

class RAGService:
    def __init__(self):
        """
        Initializes the RAGService, setting up the connection to Qdrant and the embedding client.
        """
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        openrouter_api_key = os.getenv("OPENROUTER_API_KEY")

        if not qdrant_url or not qdrant_api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in the environment.")
        
        if not openrouter_api_key:
            raise ValueError("OPENROUTER_API_KEY must be set in the environment.")

        try:
            self.qdrant_client = QdrantClient(
                url=qdrant_url, 
                api_key=qdrant_api_key,
                timeout=60,
            )
            logger.info("Successfully connected to Qdrant.")

            # Forcing deletion and recreation for this specific fix
            try:
                self.qdrant_client.delete_collection(collection_name=COLLECTION_NAME)
                logger.info(f"Deleted existing collection '{COLLECTION_NAME}' to update vector size.")
            except Exception:
                # Collection might not exist, which is fine
                pass

            self.qdrant_client.recreate_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=VectorParams(size=3072, distance=Distance.COSINE)
            )
            logger.info(f"Collection '{COLLECTION_NAME}' created with vector size 3072.")

        except Exception as e:
            logger.error(f"Failed to connect to Qdrant or manage collection: {e}")
            raise

        self.embedding_client = OpenAI(
            base_url="https://openrouter.ai/api/v1",
            api_key=openrouter_api_key,
        )
        self.embedding_model = os.getenv("OPENROUTER_EMBEDDING_MODEL", "text-embedding-3-large") # Updated model
        logger.info(f"Successfully configured OpenRouter embedding client with model: {self.embedding_model}")

    def retrieve_context(self, query: str, top_k: int = 5) -> list[str]:
        """
        Queries Qdrant to retrieve relevant document chunks for a given query.

        Args:
            query: The user's query string.
            top_k: The number of results to retrieve.

        Returns:
            A list of strings, where each string is a relevant document chunk.
        """
        # 1. Generate embedding for the query using OpenRouter embedding client
        try:
            response = self.embedding_client.embeddings.create(
                model=self.embedding_model,
                input=[query]
            )
            query_embedding = response.data[0].embedding
        except Exception as e:
            logger.error(f"Error generating embedding with OpenRouter: {e}")
            raise

        # 2. Query Qdrant
        search_result = self.qdrant_client.query_points(
            collection_name=COLLECTION_NAME,
            query=query_embedding,
            limit=top_k,
            with_payload=True,
        )

        # 3. Extract content from payloads
        # query_points returns a QueryPointsResponse with a 'points' attribute
        points = getattr(search_result, 'points', search_result)
        context = []
        for hit in points:
            # Handle both object format and tuple format
            if hasattr(hit, 'payload'):
                payload = hit.payload
            elif isinstance(hit, tuple) and len(hit) >= 2:
                # Tuple format: (id, vector, payload, ...)
                payload = hit[2] if len(hit) > 2 else {}
            else:
                continue

            if isinstance(payload, dict) and 'content' in payload:
                context.append(payload['content'])

        logger.info(f"Retrieved {len(context)} context chunks for query: '{query}'")
        return context

