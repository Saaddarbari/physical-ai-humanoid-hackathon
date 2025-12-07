# Physical AI & Humanoid Robotics Textbook

An interactive online textbook for "Physical AI & Humanoid Robotics" using Docusaurus v3. The textbook covers four core modules with simulation-based examples, RAG-powered Q&A, personalization, and Urdu translation features.

## Features

- **Four Core Modules**:
  - Module 1: ROS 2 Fundamentals
  - Module 2: Simulation with Gazebo/Unity
  - Module 3: Advanced Robotics with NVIDIA Isaac
  - Module 4: Vision-Language-Action (VLA) Systems

- **Interactive Learning**:
  - Simulation-based examples
  - RAG-powered Q&A system
  - Personalization based on user profile
  - Urdu translation support
  - Progress tracking

- **Accessibility**:
  - WCAG 2.1 AA compliance
  - Urdu translation
  - Keyboard navigation
  - Screen reader support

## Tech Stack

- **Frontend**: Docusaurus v3
- **Backend**: FastAPI
- **Database**: Neon PostgreSQL
- **Vector DB**: Qdrant
- **Authentication**: Better-Auth
- **Deployment**: Vercel

## Prerequisites

- Node.js 18+
- Python 3.11+
- Git
- Vercel CLI (for deployment)

## Setup

### 1. Clone the Repository

```bash
git clone https://github.com/your-org/physical-ai-humanoid-robotics.git
cd physical-ai-humanoid-robotics
```

### 2. Install Docusaurus Dependencies

```bash
cd docusaurus
npm install
```

### 3. Set up Environment Variables

Create a `.env` file in the root directory:

```env
# RAG Backend Configuration
RAG_API_URL=http://localhost:8000
RAG_API_KEY=your-api-key-if-required

# Database Configuration (for local development)
NEON_DB_URL=your-neon-db-url
QDRANT_URL=your-qdrant-url

# Personalization Configuration
PERSONALIZATION_ENABLED=true
```

### 4. Install Python Dependencies

```bash
pip install -r requirements.txt
```

## Running Locally

### 1. Start the Docusaurus Development Server

```bash
cd docusaurus
npm start
```

This will start the local development server and open the textbook in your browser at http://localhost:3000.

### 2. Running the RAG Backend (Optional)

If you want to test the RAG functionality locally:

```bash
# In a new terminal
pip install -r requirements.txt
uvicorn main:app --reload --port 8000
```

## Project Structure

```
├── docusaurus/           # Docusaurus site files
│   ├── src/             # Components, pages, CSS
│   ├── static/          # Static assets
│   ├── docs/            # Textbook content
│   ├── docusaurus.config.js
│   └── sidebars.js
├── backend/             # FastAPI backend (TBD)
├── docs/                # Documentation
├── scripts/             # Utility scripts
├── specs/               # Feature specifications
├── requirements.txt     # Python dependencies
├── package.json         # Node.js dependencies
└── vercel.json          # Vercel deployment config
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Commit your changes (`git commit -m 'Add amazing feature'`)
5. Push to the branch (`git push origin feature/amazing-feature`)
6. Open a Pull Request

## Deployment

This project is configured for deployment on Vercel:

1. Install Vercel CLI: `npm install -g vercel`
2. Login to Vercel: `vercel login`
3. Deploy: `vercel --prod`

## License

MIT