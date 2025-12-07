---
id: 1
title: Amend Constitution
stage: constitution
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution,documentation,setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

**Project:** AI-driven technical book built and published using Docusaurus and GitHub Pages, written using Spec-Kit Plus and Claude Code.

## Core Principles
- Accuracy and correctness in all technical explanations
- Clarity for intermediate-level software developers
- Progressive learning (chapter complexity increases gradually)
- Practical, implementation-first approach
- Human review of all AI-generated content

## Key Standards
- Each chapter must include code examples, explanations, and references
- GitHub-linked source code where applicable
- All AI models used must be acknowledged
- Content must support future updates and versioning
- Documentation format optimized for Docusaurus structure

## Content Requirements
- Project-first teaching approach
- Real working examples preferred over theoretical discussion
- Hands-on tasks included per section
- No unexplained terminology
- Visuals/diagrams recommended when necessary

## Structural Guidelines
Each chapter must contain:
- Summary
- Learning goals
- Core concepts
- Implementation section
- Additional resources
- Quiz or reflection questions

## Tooling Requirements
- Written using Spec-Kit Plus and Claude Code
- Markdown-first pipeline
- Built using Docusaurus
- Deployed on GitHub Pages

## Writing Constraints
- Tone: educational and professional
- Audience: intermediate software developers
- Reading clarity: 8th–12th grade level
- Avoid speculative or unverified claims

## Sources
Acceptable:
- Official documentation
- Academic or peer-reviewed sources
- Developer blogs
- Open-source documentation

Avoid:
- Unverified social media sources

## Success Criteria
- Docusaurus site builds and deploys successfully on GitHub Pages
- All chapters follow Spec-Kit specification
- Code examples function as written
- Logical, progressive learning flow
- Zero unexplained jargon
- Readers can independently follow and implement examples

## Response snapshot

Updated the project constitution at .specify/memory/constitution.md to reflect the AI-driven technical book project requirements. The new constitution includes six core principles: AI-Driven Technical Book Accuracy, Clarity for Intermediate Developers, Progressive Learning Structure, Practical Implementation-First Approach, Human Review of AI Content, and Docusaurus GitHub Pages Publishing. Additional sections were added for Content Requirements, Structural Guidelines, Tooling Requirements, Writing Constraints, Sources, and Success Criteria. The version was updated from 1.0.0 to 1.1.0 with a sync impact report included in an HTML comment at the top of the file.

## Outcome

- ✅ Impact: Updated constitution file with project-specific principles and requirements
- 🧪 Tests: none
- 📁 Files: 1 file modified (.specify/memory/constitution.md)
- 🔁 Next prompts: None needed
- 🧠 Reflection: Constitution now properly reflects the AI-driven technical book project requirements

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A