import os
import asyncpg
from dotenv import load_dotenv
import uuid
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Load environment variables from .env file
load_dotenv()

class DatabaseService:
    def __init__(self):
        """
        Initializes the DatabaseService, setting up the connection pool to Neon Postgres.
        """
        self.database_url = os.getenv("NEON_DATABASE_URL")
        if not self.database_url:
            raise ValueError("NEON_DATABASE_URL must be set in the environment.")
        
        self.pool = None

    async def connect(self):
        """
        Establishes the connection pool to the database and creates tables if they don't exist.
        """
        try:
            self.pool = await asyncpg.create_pool(self.database_url)
            await self.create_tables()
            logger.info("Successfully connected to Neon Postgres and created tables.")
        except Exception as e:
            logger.error(f"Failed to connect to Neon Postgres: {e}")
            raise

    async def disconnect(self):
        """
        Closes the connection pool.
        """
        if self.pool:
            await self.pool.close()
            logger.info("Successfully disconnected from Neon Postgres.")

    async def create_tables(self):
        """
        Creates the conversation and message tables if they do not exist.
        """
        async with self.pool.acquire() as connection:
            await connection.execute("""
                CREATE TABLE IF NOT EXISTS conversation (
                    id UUID PRIMARY KEY,
                    user_id TEXT NOT NULL,
                    created_at TIMESTAMP DEFAULT NOW()
                );
            """)
            await connection.execute("""
                CREATE TABLE IF NOT EXISTS message (
                    id UUID PRIMARY KEY,
                    conversation_id UUID REFERENCES conversation(id),
                    role TEXT NOT NULL,
                    content TEXT NOT NULL,
                    created_at TIMESTAMP DEFAULT NOW()
                );
            """)
            logger.info("Tables 'conversation' and 'message' are ready.")

    async def save_message(self, user_id: str, role: str, content: str):
        """
        Saves a message to the database, creating a new conversation if necessary.
        """
        async with self.pool.acquire() as connection:
            # Find conversation or create a new one
            conversation = await connection.fetchrow("SELECT * FROM conversation WHERE user_id = $1", user_id)
            if not conversation:
                conversation_id = uuid.uuid4()
                await connection.execute(
                    "INSERT INTO conversation (id, user_id) VALUES ($1, $2)",
                    conversation_id, user_id
                )
                logger.info(f"Created new conversation for user_id: {user_id}")
            else:
                conversation_id = conversation['id']

            # Insert the new message
            await connection.execute(
                "INSERT INTO message (id, conversation_id, role, content) VALUES ($1, $2, $3, $4)",
                uuid.uuid4(), conversation_id, role, content
            )
            logger.info(f"Saved message for user_id: {user_id}")

    async def get_history(self, user_id: str) -> list[dict]:
        """
        Retrieves the message history for a given user.
        """
        async with self.pool.acquire() as connection:
            conversation = await connection.fetchrow("SELECT * FROM conversation WHERE user_id = $1", user_id)
            if not conversation:
                return []
            
            messages = await connection.fetch(
                "SELECT role, content FROM message WHERE conversation_id = $1 ORDER BY created_at",
                conversation['id']
            )
            logger.info(f"Retrieved {len(messages)} messages for user_id: {user_id}")
            return [dict(m) for m in messages]

    async def delete_history(self, user_id: str) -> bool:
        """
        Deletes the entire conversation history for a given user.
        """
        async with self.pool.acquire() as connection:
            # Find conversation
            conversation = await connection.fetchrow("SELECT * FROM conversation WHERE user_id = $1", user_id)
            if not conversation:
                logger.warning(f"Attempted to delete history for non-existent user_id: {user_id}")
                return False

            conversation_id = conversation['id']
            
            # Delete messages and conversation
            async with connection.transaction():
                await connection.execute("DELETE FROM message WHERE conversation_id = $1", conversation_id)
                await connection.execute("DELETE FROM conversation WHERE id = $1", conversation_id)
            
            logger.info(f"Deleted history for user_id: {user_id}")
            return True
