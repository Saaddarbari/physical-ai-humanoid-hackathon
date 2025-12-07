# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Prerequisites

Before getting started with the Physical AI & Humanoid Robotics textbook project, ensure your system meets the following requirements:

### System Requirements
- **Operating System**: macOS, Linux, or Windows with WSL2
- **Node.js**: Version 18+ with npm
- **Python**: Version 3.11+ for ROS 2 examples (optional)
- **Git**: For version control and content management
- **Vercel CLI**: For deployment (install with `npm install -g vercel`)

### Cloud Services (Free Tier)
- **Neon**: For PostgreSQL database (free tier available)
- **Qdrant**: For vector database (free tier available)
- **Vercel**: For deployment and hosting (free tier available)

## Installation

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
Create a `.env` file in the docusaurus directory:
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

## Running the Textbook Locally

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
cd backend
pip install -r requirements.txt
uvicorn main:app --reload --port 8000
```

## Textbook Structure

The textbook is organized into four core modules:

### Module 1: ROS 2 Fundamentals (Chapters 1-3)
- Chapter 1: Introduction to ROS 2 and Middleware
- Chapter 2: Nodes, Topics, Services, and Actions
- Chapter 3: URDF and Robot Description

### Module 2: Simulation with Gazebo/Unity (Chapters 4-6)
- Chapter 4: Physics Simulation Fundamentals
- Chapter 5: Sensor Simulation and Integration
- Chapter 6: Unity Integration for Advanced Visualization

### Module 3: Advanced Robotics with NVIDIA Isaac (Chapters 7-8)
- Chapter 7: Perception and Navigation Systems
- Chapter 8: Sim-to-Real Transfer Techniques

### Module 4: Vision-Language-Action (VLA) Systems (Chapters 9-10)
- Chapter 9: Voice-to-Action Processing
- Chapter 10: Cognitive Planning with LLMs

## Using the RAG-Powered Q&A

The textbook includes an AI-powered Q&A system to help clarify concepts:

1. Navigate to any chapter
2. Use the "Ask a Question" button
3. Type your question about the content
4. Receive an answer based on the textbook material

## Personalization Features

### Profile Setup
1. Create an account or log in
2. Complete the signup quiz to help personalize your learning experience
3. Set your accessibility preferences (including Urdu translation)

### Urdu Translation
- Toggle Urdu translation using the language switcher in the header
- Note: Translation is currently in development and may not be available for all content

## Contributing Content

### Adding New Chapters
1. Create a new Markdown file in the appropriate module directory
2. Follow the existing chapter template
3. Add the chapter to the sidebar configuration
4. Ensure all learning objectives are met

### Adding Simulation Examples
1. Place ROS 2 simulation files in the `simulation/` directory
2. Reference the simulation in your chapter content using the appropriate Docusaurus component
3. Test the simulation locally before committing

## Deployment to Vercel

### 1. Install Vercel CLI
```bash
npm install -g vercel
```

### 2. Login to Vercel
```bash
vercel login
```

### 3. Deploy to Vercel
```bash
cd docusaurus
vercel --prod
```

Alternatively, you can link your GitHub repository to Vercel for automatic deployments on push.

## Troubleshooting

### Common Issues

**Issue**: Docusaurus site not starting
**Solution**: Ensure Node.js 18+ is installed and run `npm install` to reinstall dependencies

**Issue**: RAG functionality not working
**Solution**: Verify that the RAG backend is running and environment variables are correctly set

**Issue**: Urdu translation not appearing
**Solution**: Check that translation files are properly formatted and the language switcher is enabled

**Issue**: Vercel deployment failing
**Solution**: Verify Vercel CLI is installed and you're logged in, check environment variables are properly set

### Getting Help

For additional support:
1. Check the documentation at [your documentation URL]
2. Open an issue in the repository
3. Join our community Discord (if applicable)

## Next Steps

1. Complete the signup quiz to enable personalization
2. Start with Module 1: ROS 2 Fundamentals
3. Experiment with the RAG Q&A system
4. Explore simulation examples
5. Enable Urdu translation if needed