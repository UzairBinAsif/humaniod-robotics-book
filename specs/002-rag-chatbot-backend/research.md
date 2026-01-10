# Research & Decisions for Integrated RAG Chatbot

This document records the decisions made to resolve points of ambiguity identified in the planning phase.

## 1. Frontend Interaction for Contextual Questions

- **Context**: The `plan.md` identified a need to clarify how a user would select text on the frontend to be passed to the backend API's `selected_text` field.
- **Decision**: A lightweight JavaScript approach will be the recommended implementation for the frontend. When a user finishes highlighting text within the book's content, a small, non-intrusive popup or button with a label like "Ask about this" will appear near the selection. Clicking this button will capture the selected text and associate it with the user's next query in the chat interface.
- **Rationale**: This approach provides a clear, contextual affordance to the user without requiring a heavy frontend framework. It is a common and intuitive pattern. It directly maps to the backend API which is designed to accept a `selected_text` field, ensuring a clean separation of concerns.
- **Alternatives Considered**: 
    - A persistent side panel that automatically shows selected text was considered but rejected. This would require more significant frontend work and consume more screen real-estate, which is outside the scope of this backend-focused feature.
    - A custom right-click context menu was also rejected due to the complexity of creating a consistent and reliable cross-browser experience.

## 2. Data Privacy and History Management

- **Context**: The plan requires storing chat history in a Neon Postgres database, but the specific data privacy policies were not defined.
- **Decision**: The system will be designed with user privacy in mind. An API endpoint (e.g., `DELETE /history/{user_id}`) will be included to allow a user to delete their entire conversation history. For the initial implementation, data will not be explicitly anonymized but will be tied to a non-personally-identifiable `user_id` (e.g., a session ID or a randomly generated UUID).
- **Rationale**: This aligns with common data privacy principles like GDPR's "right to be forgotten" and provides users with direct control over their data. This approach represents a good balance between providing conversation history functionality and respecting user privacy for an initial version.
- **Alternatives Considered**: 
    - Storing data indefinitely with full anonymization was rejected because it removes user agency and can be complex to implement correctly.
    - Implementing full, region-specific compliance (like full GDPR articles) from the start was deemed too complex for the initial feature scope and can be added later if required.

## 3. Book Content Ingestion into Qdrant

- **Context**: The plan specified using Qdrant as a vector store but did not detail the ingestion process.
- **Decision**: A one-time, standalone Python script will be developed for content ingestion. This script will perform the following steps:
    1. Read the entire book content from a specified source file (e.g., a large `.txt` or `.md` file).
    2. Split the content into meaningful chunks (e.g., by paragraph or a fixed token window with overlap).
    3. Use the specified Gemini model (via the OpenAI-compatible endpoint) to generate vector embeddings for each chunk.
    4. Connect to the configured Qdrant Cloud collection and upload the chunks with their corresponding embeddings.
- **Rationale**: This is the simplest and most direct approach for an initial implementation. It decouples the complex ingestion process from the live API, ensuring the chatbot can be deployed and run without the added complexity of a live ingestion pipeline. For content that is largely static (like a published book), this is a highly effective strategy.
- **Alternatives Considered**: 
    - An automated, incremental pipeline was rejected as it adds significant complexity (e.g., file watching, versioning, delta computations) not required for a book with static or infrequently updated content.
    - Dynamic ingestion from a CMS was rejected as no such source was specified, and it would introduce an external dependency.
