---
id: 1
title: amend-constitution-vercel-deployment
stage: constitution
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: claude
command: /sp.constitution
labels: [constitution,vercel,deployment,docusaurus]
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

**Project:** AI-driven technical book built and published using Docusaurus and vercel, written using Spec-Kit Plus and Claude Code.

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
- Deployed on vercel

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
- Docusaurus site builds and deploys successfully on vercel
- All chapters follow Spec-Kit specification
- Code examples function as written
- Logical, progressive learning flow
- Zero unexplained jargon
- Readers can independently follow and implement examples

## Response snapshot

Updated the project constitution to reflect Vercel deployment requirement instead of GitHub Pages. Updated version from 1.1.0 to 1.2.0 and changed the sixth principle from "Docusaurus GitHub Pages Publishing" to "Docusaurus Vercel Publishing". Updated all references to deployment platform throughout the document to reflect Vercel instead of GitHub Pages.

## Outcome

- ✅ Impact: Updated constitution to reflect Vercel deployment requirement
- 🧪 Tests: none
- 📁 Files: 1 file updated (.specify/memory/constitution.md)
- 🔁 Next prompts: none
- 🧠 Reflection: Successfully updated constitution to match user requirements

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none