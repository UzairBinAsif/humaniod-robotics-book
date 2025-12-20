# Research: Technology Choices for Humanoid Robotics Book

This document records the decisions made to resolve ambiguities identified in the technical plan.

## 1. Vision-Language Model (VLM) Selection

-   **Decision**: Use a pre-trained LLaVA (Large Language and Vision Assistant) model, specifically a 7B parameter version optimized for inference.
-   **Rationale**:
    -   **State-of-the-Art**: LLaVA represents a standard and effective architecture for vision-language tasks.
    -   **Accessibility**: Pre-trained 7B models are readily available on platforms like HuggingFace and can run on consumer-grade NVIDIA GPUs, which aligns with the project's hardware constraints.
    -   **Community Support**: LLaVA has strong community support and numerous examples, making it suitable for a book aimed at developers new to the field.
-   **Alternatives Considered**:
    -   **GPT-4 with Vision**: More powerful, but requires a paid API, which adds a barrier to entry for readers. The goal is to rely on open, locally-runnable models where possible.
    -   **Building a custom VLM**: Out of scope for this project. The focus is on *integrating* existing models, not training them from scratch.
