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

## Architecture Sketch

High-level components:

1. **Docusaurus Site** – Frontend for the textbook content, navigation, search, and presentation of chapters.
2. **RAG Backend** – FastAPI application serving RAG queries, using Neon (Postgres) for structured data and Qdrant (vector DB) for semantic search.
3. **Auth/Personalization Layer** – Allows user authentication and personalized recommendations based on a signup quiz.
4. **Translation Module** – Handles Urdu translation via client-side plugin or potential backend integration.
5. **Simulation Framework** – ROS 2, Gazebo, Unity, and NVIDIA Isaac simulations integrated for hands-on exercises.

---

## Section Structure

- **Introduction:** Overview of Physical AI & Humanoid Robotics
- **Module 1: ROS 2 Fundamentals** (Chapters 1–3)
- **Module 2: Simulation with Gazebo/Unity** (Chapters 4–6)
- **Module 3: Advanced Robotics with NVIDIA Isaac** (Chapters 7–8)
- **Module 4: Vision-Language-Action (VLA) Systems** (Chapters 9–10)
- **Conclusion:** Future outlook and challenges

Each chapter includes:
- **Personalization:** Optional tips or adaptive content based on user profile
- **Urdu Translation:** Toggle for chapter content

---

## Research Approach

- Conduct concurrent research while writing each module
- Gather 5+ authoritative sources per module (academic papers, official docs, tutorials)
- Prioritize resources relevant to hands-on exercises and conceptual understanding
- Document decisions on research approach vs. upfront preparation

---

## Quality Validation

- **RAG Accuracy:** ≥90% on a test set of 20 queries
- **Deployment:** Fully functional Docusaurus site on Vercel
- **User Flow Simulation:** Test signup quiz, personalization, translation toggles
- **Accessibility:** WCAG 2.1 AA compliance

---

## Implementation Phases

1. **Phase 1 – Core Book Structure & Docusaurus Setup**
   - Complete Docusaurus setup with placeholder content
   - Establish chapter/module structure
   - Implement basic Vercel deployment workflow

2. **Phase 2 – RAG Integration**
   - Setup FastAPI backend, Neon, Qdrant
   - Develop RAG query processing
   - Ensure database ready before embedding

3. **Phase 3 – Bonus Features**
   - User authentication via Better-Auth
   - Personalization logic based on quiz/profile
   - Urdu translation module
   - Subagents for future interactive elements

4. **Phase 4 – Testing & Deployment Refinement**
   - Comprehensive feature testing (RAG, personalization, translation)
   - WCAG accessibility testing
   - Optimize Vercel deployment

---

## Dependencies

- Docusaurus setup must precede chapter content population
- RAG database configured before embedding/querying
- Bonus features can be implemented sequentially

---

## Decisions Needing Documentation

1. **RAG Vector Database**
   - **Options:** Qdrant vs. in-memory vector store (FAISS)
   - **Trade-offs:** Qdrant is scalable, persistent; FAISS simpler but limited
   - **Choice:** Qdrant for free-tier production-like RAG

2. **Research Approach for Modules**
   - **Options:** Concurrent vs. upfront
   - **Trade-offs:** Concurrent is agile, upfront ensures consistency
   - **Choice:** Concurrent to maintain iterative workflow

3. **Personalization Depth**
   - **Options:** Simple tips vs. dynamic content rewrite
   - **Trade-offs:** Simple tips easier to implement, dynamic more complex
   - **Choice:** Start with simple tips, expand later

4. **Translation Module Implementation**
   - **Options:** Client-side plugin, server-side integration, third-party API
   - **Trade-offs:** Client-side simpler, server-side more control, third-party costly
   - **Choice:** Client-side initially, potential future server-side upgrade

5. **Deployment Platform**
   - **Options:** GitHub Pages vs. Vercel vs. Netlify
   - **Trade-offs:** Vercel offers better CI/CD, serverless functions, and performance; GitHub Pages is simpler but more limited
   - **Choice:** Vercel for superior deployment features and scalability

---

## Technical Details

- **License:** MIT
- **Accessibility:** WCAG 2.1 AA compliance
- **Cost:** Free-tier cloud services only
- **Hardware:** Simulation-based examples only

---

## Follow-ups & Risks

- **Follow-up:** Confirm free-tier limits and scalability for Neon & Qdrant
- **Risk:** RAG backend performance on free-tier
- **Follow-up:** Evaluate Docusaurus plugins for personalization & translation
- **Risk:** Urdu translation quality without high-cost APIs
- **Follow-up:** Content versioning strategy for emerging robotics technologies
- **Risk:** Keeping content up-to-date with rapid AI/humanoid robotics advances