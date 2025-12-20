---
id: 1
title: Establish Robotics Constitution
stage: constitution
date: 2025-12-20
surface: agent
model: gemini-cli
feature: none
branch: master
user: unknown
command: /sp.constitution Act as a Senior Robotics Engineer and AI Researcher. Your tone is technical yet accessible. Follow Docusaurus MD formatting. Focus on the integration of NVIDIA Isaac and ROS 2 for Humanoid applications.
labels: [robotics, constitution, ros2, nvidia-isaac]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  []
---

## Prompt

/sp.constitution Act as a Senior Robotics Engineer and AI Researcher. Your tone is technical yet accessible. Follow Docusaurus MD formatting. Focus on the integration of NVIDIA Isaac and ROS 2 for Humanoid applications.

## Response snapshot

I have established the initial project constitution at `.specify/memory/constitution.md`.

**Version:** 1.0.0
**Rationale:** This version sets the foundation for our development process, codifying our commitment to a ROS 2-centric architecture, leveraging the NVIDIA Isaac platform for simulation and AI, and enforcing strict standards for safety and modularity.

**Manual Follow-up Required:**
The following template files may need updates to align with the new constitution:
-   `.specify/templates/plan-template.md`
-   `.specify/templates/spec-template.md`
-   `.specify/templates/tasks-template.md`

**Suggested Commit Message:**
```
docs: establish v1.0.0 of project constitution
```

## Outcome

- ✅ Impact: Established the foundational project constitution, defining core technical principles and governance.
- 🧪 Tests: No tests were run.
- 📁 Files: Created the initial constitution file.
- 🔁 Next prompts: Review and align templates (`plan`, `spec`, `tasks`) with the new constitution.
- 🧠 Reflection: The process was smooth. The script for PHR creation was missing, so I had to fall back to the manual method.

## Evaluation notes (flywheel)

- Failure modes observed: PHR creation script not found.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure PHR creation scripts are present and executable in the environment.
