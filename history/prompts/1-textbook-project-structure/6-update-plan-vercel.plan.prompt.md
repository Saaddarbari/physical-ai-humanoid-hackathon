---
id: 6
title: Update Plan to Vercel
stage: plan
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-textbook-project-structure
branch: 1-textbook-project-structure
user: user
command: /sp.plan
labels: [plan,ai,robotics,education,docusaurus,vercel]
links:
  spec: specs/1-textbook-project-structure/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/1-textbook-project-structure/plan.md
 - specs/1-textbook-project-structure/research.md
 - specs/1-textbook-project-structure/quickstart.md
tests:
 - none
---

## Prompt

# Implementation Plan: Physical AI & Humanoid Robotics Textbook Project

**Feature Branch**: `1-textbook-project-structure`
**Created**: 2025-12-07
**Spec**: [link to spec.md](./spec.md)
**Input**: Feature specification from `/specs/1-textbook-project-structure/spec.md`

**Note**: This plan follows the `/sp.plan` workflow with architecture sketch, section structure, research approach, and quality validation.

---

## Summary

Develop an interactive online textbook for "Physical AI & Humanoid Robotics" using **Docusaurus v3**. The textbook will cover four core modules:

1. **Module 1:** ROS 2 Fundamentals
2. **Module 2:** Simulation with Gazebo/Unity
3. **Module 3:** Advanced Robotics with NVIDIA Isaac
4. **Module 4:** Vision-Language-Action (VLA) Systems

Content will include simulation-based examples, optional RAG-powered Q&A, personalization, and Urdu translation features. The project targets accessibility, scalability, and modular content delivery. Hands-on exercises will use simulations instead of physical robots.

Deployment will be done on **Vercel**, ensuring automatic hosting and CI/CD integration.

---

## Technical Context

- **Language/Version:** Markdown for content, JavaScript/TypeScript for Docusaurus site, Python 3.11+ for ROS 2 and simulation examples
- **Dependencies:** Docusaurus 3.x, Node.js 18+, FastAPI (RAG backend), Neon (Postgres), Qdrant (vector DB)
- **Storage:** Git repository for source content
- **Deployment:** **Vercel** (replacing GitHub Pages)
- **Testing:** Simulation-based validation, automated content checks, RAG test queries
- **Target Platform:** Web (desktop/mobile), Ubuntu 22.04 LTS for local simulation examples
- **Project Type:** Interactive online textbook + optional simulation framework
- **Performance Goals:** Fast navigation (<200ms), RAG answers ≥90% accurate, scalable for 1000+ concurrent users
- **Constraints:** Free-tier cloud services only, offline-capable content delivery, WCAG 2.1 AA accessibility compliance

---

## Constitution Check

- ✅ Accurate technical content
- ✅ Clear for intermediate developers
- ✅ Progressive learning structure across four modules
- ✅ Practical examples with simulation-first approach
- ✅ Human review before deployment
- ✅ Optimized for Docusaurus & **Vercel**

---

## Project Structure

```text
specs/1-textbook-project-structure/
├── plan.md          # This file
├── research.md      # Phase 0 output
├── data-model.md    # Phase 1 output
├── quickstart.md    # Phase 1 output
├── contracts/       # Phase 1 output
└── tasks.md         # Phase 2 output

docs/
├── intro.md
├── modules/
│   ├── module1-ros2/
│   ├── module2-gazebo-unity/
│   ├── module3-isaac/
│   └── module4-vla/
├── tutorials/
├── api/
└── guides/

docusaurus/
├── src/
│   ├── components/
│   ├── pages/
│   └── css/
├── static/
├── docs/
└── sidebars.js

scripts/
├── setup.sh
├── build.sh
└── deploy.sh
```

## Response snapshot

Successfully updated the implementation plan for the Physical AI & Humanoid Robotics Textbook Project to reflect the decision to deploy on Vercel instead of GitHub Pages. Updated artifacts include:

1. Implementation plan (plan.md) - Updated to reflect Vercel deployment, updated project structure, and added deployment platform decision
2. Research summary (research.md) - Added deployment platform decision with rationale and alternatives
3. Quickstart guide (quickstart.md) - Updated deployment instructions to use Vercel CLI with proper setup and troubleshooting

The updates maintain all existing functionality while incorporating the new deployment strategy that provides superior CI/CD integration, serverless functions, better performance, and enhanced scalability compared to GitHub Pages.

## Outcome

- ✅ Impact: Updated plan to use Vercel for deployment instead of GitHub Pages
- 🧪 Tests: none
- 📁 Files: 3 files updated to reflect Vercel deployment
- 🔁 Next prompts: Ready for task breakdown with updated deployment strategy
- 🧠 Reflection: Plan now accurately reflects Vercel deployment decision

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed with task breakdown using Vercel deployment