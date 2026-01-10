# Feature Specification: Integrated RAG Chatbot

**Feature Branch**: Not applicable (User requested to bypass git)
**Created**: 2025-12-29
**Status**: Draft
**Input**: User description: "Create a 'backend' directory in the project root. Build an Integrated RAG Chatbot using Gemini 3 Flash and the OpenAI Agents SDK inside this directory. Stack: FastAPI, Neon Serverless Postgres (history), and Qdrant Cloud (vector DB). Feature: Users can ask questions about the book, including context-based questions on selected text. Use the Gemini OpenAI-compatible endpoint."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask General Questions (Priority: P1)

A user asks a general question about any topic covered in the book. The chatbot provides an accurate and relevant answer based on the book's content.

**Why this priority**: Core functionality, provides immediate value to the user by enabling basic information retrieval from the book.

**Independent Test**: Can be fully tested by typing a question and verifying the received answer's relevance and accuracy against the book's content.

**Acceptance Scenarios**:

1.  **Given** the chatbot is active and the book's content is indexed, **When** a user asks "What is ROS2?", **Then** the chatbot provides a summary of ROS2 based on the book's information.
2.  **Given** the chatbot is active, **When** a user asks a question about a concept not covered in the book, **Then** the chatbot indicates it cannot find an answer within its knowledge base.

---

### User Story 2 - Ask Context-Based Questions (Priority: P1)

A user selects a specific passage of text from the book and asks a question directly related to that selected context. The chatbot uses the selected text to refine its answer, providing more precise and relevant information.

**Why this priority**: This differentiates the RAG chatbot from a simple QA system, enabling deep contextual understanding crucial for detailed academic or technical inquiry.

**Independent Test**: Can be fully tested by selecting various text passages, asking relevant questions, and validating that the chatbot's answers specifically address the selected context.

**Acceptance Scenarios**:

1.  **Given** the chatbot is active and the user has selected a paragraph describing URDF, **When** the user asks "What does 'link' refer to here?", **Then** the chatbot's answer specifically explains 'link' in the context of URDF as described in the selected text.
2.  **Given** the chatbot is active and the user has selected a sentence, **When** the user asks a question unrelated to the selected text, **Then** the chatbot either indicates it cannot use the selected text to answer the question or provides an answer from the general knowledge base, clarifying that the selected text was not used.

---

### User Story 3 - Conversation History (Priority: P2)

The chatbot maintains a history of the user's conversation, allowing for follow-up questions and continuity across interactions, enhancing the user experience.

**Why this priority**: Improves user experience by allowing seamless continuation of thought processes and reducing repetitive input, leading to more efficient information retrieval.

**Independent Test**: Can be fully tested by engaging in a multi-turn conversation, closing and reopening the chat interface, and verifying the persistence and contextual awareness of the conversation.

**Acceptance Scenarios**:

1.  **Given** a user has engaged in multiple question-and-answer turns, **When** they later access the chat interface, **Then** their previous conversation history is displayed chronologically.
2.  **Given** a user has asked "What is ROS2?" and received an answer, **When** they then ask "How is it different from ROS1?" in the same conversation, **Then** the chatbot uses the context of the previous question to provide a comparative answer.

---

### Edge Cases

-   **Irrelevant/Short Selected Text**: What happens when the selected text is too short, generic, or completely irrelevant to the user's question?
-   **Long/Complex Questions**: How does the system handle very long, ambiguous, or multi-part questions?
-   **No Answer Found**: What is the user experience when no relevant answer can be found within the book's knowledge base for a given question?

## Requirements *(mandatory)*

### Functional Requirements

-   **FR-001**: The system MUST provide an intuitive interface for users to submit questions to the RAG chatbot.
-   **FR-002**: The system MUST ingest and process the book's content to build an accurate and searchable knowledge base.
-   **FR-003**: The system MUST generate answers to user questions based *exclusively* on information contained within the ingested book content.
-   **FR-004**: The system MUST be capable of identifying and utilizing specific user-selected text as supplementary context for a submitted question.
-   **FR-005**: The system MUST persistently store and enable the retrieval of a history of user conversations.
-   **FR-006**: The system MUST clearly indicate to the user when a relevant answer cannot be found within the book's knowledge base.
-   **FR-007**: The system MUST present all chatbot responses in a clear, concise, and easily understandable human-readable format.
-   **FR-008**: The system MUST enable users to specify a relevant segment of the text as additional context for their queries. The user interface for this will be a small "Ask" popup that appears when a user highlights text.
-   **FR-009**: The system MUST manage the storage and retrieval of conversation history, and MUST provide a mechanism for users to delete their entire conversation history.
-   **FR-010**: The system MUST allow for the book's content to be ingested via a one-time manual process. Future updates may require a re-run of this process.

### Key Entities *(include if feature involves data)*

-   **User**: Represents the individual interacting with the RAG chatbot. Key attributes include a unique identifier for conversation tracking.
-   **BookContent**: Refers to the entire text of the book, which is processed and chunked into smaller, semantically meaningful units.
-   **Query**: A question posed by the user. It may optionally include `SelectedText` for contextualization.
-   **SelectedText**: A specific passage or segment highlighted by the user from the `BookContent`, intended to provide additional context for a `Query`.
-   **Answer**: The response generated by the RAG system, derived from the `BookContent` and influenced by the `Query` and `SelectedText`.
-   **Conversation**: A chronological sequence of interactions (pairs of `Queries` and `Answers`) associated with a specific `User`.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: 90% of factual questions submitted to the chatbot about the book's content receive an accurate and verifiable answer derived solely from the book.
-   **SC-002**: For context-based questions (where user-selected text is provided), the system's response correctly utilizes the provided text to inform and refine the answer in 95% of cases.
-   **SC-003**: The median response time for any user query (general or context-based) is less than 3 seconds.
-   **SC-004**: 80% of users successfully find an answer to their primary question within a session, without needing to rephrase their question more than twice.
-   **SC-005**: The RAG chatbot service maintains an availability of 99.9% during identified peak usage hours.
