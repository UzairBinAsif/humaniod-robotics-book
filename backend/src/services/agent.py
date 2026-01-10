import os
import logging
from typing import Optional, List
from openai import OpenAI
from dotenv import load_dotenv

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Load environment variables from .env file
load_dotenv()


class AgentService:
    """
    Agent service using the OpenRouter API.
    """

    def __init__(self):
        """
        Initializes the AgentService with the OpenRouter client.
        """
        openrouter_api_key = os.getenv("OPENROUTER_API_KEY")

        if not openrouter_api_key:
            raise ValueError("OPENROUTER_API_KEY must be set in the environment.")

        # Configure client for OpenRouter API
        self.client = OpenAI(
            base_url="https://openrouter.ai/api/v1",
            api_key=openrouter_api_key,
        )

        self.model = "google/gemini-2.0-flash-001"
        logger.info(f"AgentService initialized for OpenRouter with model: {self.model}")

    def _build_prompt(self, query: str, context: List[str], selected_text: Optional[str] = None) -> str:
        """
        Builds the prompt for the language model.
        """
        if selected_text:
            context_str = f"""
The user has highlighted this specific text from the book:
---
{selected_text}
---
"""
        else:
            context_items = "\n".join(f"- {item}" for item in context)
            context_str = f"""
Here is the most relevant context we found in the book:
---
{context_items}
---
"""

        prompt = f"""{context_str.strip()}

Based on the provided context, please answer the following question:

User's question: {query}
"""
        return prompt.strip()

    def _transform_history(self, history: List[dict]) -> List[dict]:
        """
        Transforms the database history to OpenAI chat format.
        """
        transformed_history = []
        for msg in history:
            role = msg.get("role")
            content = msg.get("content")

            if role and content:
                transformed_history.append({
                    "role": role,
                    "content": content
                })
        return transformed_history

    def invoke(
        self,
        query: str,
        context: List[str],
        selected_text: Optional[str] = None,
        history: Optional[List[dict]] = None
    ) -> str:
        """
        Invokes the language model with a query and context to generate a response.

        Args:
            query: The user's query string.
            context: A list of strings representing the retrieved context.
            selected_text: Optional user-selected text for focused context.
            history: Optional list of previous messages in the conversation.

        Returns:
            The generated response from the language model.
        """
        system_instruction = "You are an expert on the Humanoid Robotics Book. Your role is to answer the user's question based *only* on the provided context."

        # Build the prompt
        prompt = self._build_prompt(query, context, selected_text)

        # Prepare messages
        messages = []

        if history:
            transformed_history = self._transform_history(history)
            messages.extend(transformed_history)

        # Add user message with context
        user_message = prompt
        messages.append({"role": "user", "content": user_message})

        # Add system message
        messages.insert(0, {"role": "system", "content": system_instruction})

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.7,
                max_tokens=1024,
            )

            response_content = response.choices[0].message.content
            logger.info(f"Successfully invoked agent for query: '{query}'")
            return response_content

        except Exception as e:
            logger.error(f"Error invoking agent: {e}")
            raise
