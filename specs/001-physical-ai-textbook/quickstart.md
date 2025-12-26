# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- **Node.js**: Version 18.x or higher
- **Python**: Version 3.11 or higher
- **Git**: Latest version
- For Isaac Sim content: NVIDIA RTX 4070 Ti (12GB VRAM) or higher with CUDA support

### Development Tools
- **IDE**: VS Code with Python and TypeScript extensions
- **Docker**: For containerized development (optional but recommended)
- **npm/yarn**: Package managers for frontend dependencies

## Installation

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/physical-ai-textbook.git
cd physical-ai-textbook
```

### 2. Setup Frontend (Docusaurus)
```bash
# Navigate to frontend directory
cd frontend

# Install dependencies
npm install

# Create environment file
cp .env.example .env.local

# Update environment variables in .env.local:
# REACT_APP_AUTH_URL=http://localhost:3002
```

### 3. Setup Content Tools (Python)
```bash
# Navigate to content tools directory
cd ../content-tools

# Create and activate virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 4. Setup Auth Service (Better Auth)
```bash
# Navigate to auth service directory
cd ../auth-service

# Install dependencies
npm install

# Create environment file
cp .env.example .env

# Update environment variables in .env:
# DATABASE_URL=your_postgres_connection_string
# BETTER_AUTH_SECRET=your_secret_key
# BETTER_AUTH_URL=http://localhost:3002
# FRONTEND_URL=http://localhost:3000
# PORT=3002
```

## Running the Application

### 1. Start Auth Service
```bash
cd auth-service
npm run dev
```

### 2. Start Frontend (Docusaurus)
```bash
cd frontend
npm start
```

The application will be available at:
- Frontend: http://localhost:3000
- Auth Service: http://localhost:3002

## Adding Content

### Creating a New Chapter
1. Create a new MDX file in the appropriate part directory following CHAPTERS_STRUCTURE.md:
   ```
   frontend/docs/part-1-foundations-lab/chapter-XX-topic-name.mdx
   ```

2. Follow the required structure in the file:
   ```mdx
   ---
   title: "Chapter Title"
   description: "Brief description of the chapter"
   learning_outcomes:
     - "Outcome 1"
     - "Outcome 2"
     - "Outcome 3"
   prerequisites:
     - "Prerequisite knowledge"
   ---

   ## Intro
   [Introduction to the topic]

   ## Learning Objectives
   [3-5 measurable outcomes based on Bloom's Taxonomy]

   ## Hook
   [Engaging element to capture attention]

   ## Concept
   [Core concept explanation]

   ## Mermaid Diagram
   ```mermaid
   [Mermaid diagram code]
   ```

   ## Code Example
   ```python
   # Code example with explanations
   ```

   ## Exercises
   [Hands-on exercises for the learner]

   ## Summary
   [Key points summary]

   ## Preview Next Chapter
   [Brief preview of upcoming content]
   ```

3. Update `sidebars.ts` to include the new chapter in navigation following the 5-part structure

### Validating Content
```bash
cd content-tools
source .venv/bin/activate
python scripts/validate_content.py
```

## Development Commands

### Frontend Commands
```bash
npm start          # Start development server
npm run build      # Build for production
npm run serve      # Serve production build locally
npm run docusaurus # Run docusaurus commands directly
```

### Content Commands
```bash
# Validate all code examples in chapters
cd content-tools
source .venv/bin/activate
python scripts/validate_content.py

# Generate content from templates
python scripts/generate_chapters.py

# Check for broken links in documentation
npm run docusaurus check-links
```

## Environment Configuration

### Required Environment Variables

#### Frontend (.env.local)
```
REACT_APP_AUTH_URL=http://localhost:3002
```

#### Auth Service (.env)
```
DATABASE_URL=postgresql://user:pass@localhost/dbname
BETTER_AUTH_SECRET=your_secret_key
BETTER_AUTH_URL=http://localhost:3002
FRONTEND_URL=http://localhost:3000
PORT=3002
```

## Testing

### Frontend Testing
```bash
npm test
npm run test:watch    # Run tests in watch mode
```

### Content Validation
```bash
# Validate all code examples in chapters
cd content-tools
source .venv/bin/activate
python scripts/validate_content.py

# Check for broken links in documentation
npm run docusaurus check-links

# Validate chapter structure
python scripts/validate_content.py --check-structure
```

## Deployment

### Frontend (Vercel)
```bash
# Build the application
npm run build

# Deploy using Vercel CLI
vercel --prod
```

### Auth Service (Railway)
```bash
# Deploy using Railway CLI
railway up
```

## Troubleshooting

### Common Issues

1. **Port already in use**: Change PORT in .env files if default ports are taken
2. **Python dependencies**: Ensure virtual environment is activated when running content tools
3. **Missing environment variables**: Check all .env files are properly configured
4. **Content not appearing**: Verify chapter is added to `sidebars.ts` navigation

### Performance Tips
- Use NVIDIA GPU for Isaac Sim content
- Increase Node.js memory limit for large documentation builds: `export NODE_OPTIONS="--max_old_space_size=4096"`
- Follow CHAPTERS_STRUCTURE.md for consistent content organization