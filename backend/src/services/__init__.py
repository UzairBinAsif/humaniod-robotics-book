# src/services/__init__.py

"""
This file centralizes the instantiation of all services to prevent circular import errors.
By importing the classes and creating the singleton instances here, other modules
can import the instances directly from the `services` package.
"""

from .agent import AgentService
from .db import DatabaseService
from .rag_service import RAGService

# Create singleton instances of each service
agent_service = AgentService()
db_service = DatabaseService()
rag_service = RAGService()
