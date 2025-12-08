# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `001-physical-ai-textbook` | **Date**: 2025-12-05 | **Spec**: [specs/001-physical-ai-textbook/spec.md](specs/001-physical-ai-textbook/spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-textbook/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Development of a comprehensive, open-source technical textbook for Physical AI & Humanoid Robotics following the CHAPTERS_STRUCTURE.md. The textbook will deliver 5 parts (13 weeks) of educational content with 28 chapters, structured with Intro → Learning Objectives → Hook → Concept → Mermaid Diagram → Code Example → Exercises → Summary → Preview Next Chapter format. The content will be delivered as MDX files in a Docusaurus-based educational platform with pedagogical scaffolding for different learning styles and hardware capabilities.

## Technical Context

**Language/Version**: TypeScript 5.x (Docusaurus frontend), Python 3.11+ (ROS 2 Humble), Python 3.12+ (backend tools)
**Primary Dependencies**: Docusaurus 3.x, rclpy (ROS 2 Humble), Node.js, MDX processing tools
**Storage**: Git-based (content storage), Neon Postgres (user data and metadata)
**Testing**: pytest (Python tools), Jest/React Testing Library (TypeScript frontend), manual QA for content validation
**Target Platform**: Web-based (Docusaurus on Vercel), with simulation content requiring NVIDIA RTX 4070 Ti or higher for Isaac Sim
**Project Type**: Educational content platform (frontend with textbook content)
**Performance Goals**: Docusaurus build time < 2 minutes, content validation < 30 seconds per chapter
**Constraints**: NVIDIA RTX 4070 Ti+ required for Isaac Sim content, hardware requirements vary by user tier (A: laptop-only, B: Jetson, C: humanoid robot)
**Scale/Scope**: Support for 3 user tiers (A: laptop-only, B: Jetson Orin Nano, C: humanoid robot), 28 chapters across 5 parts

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Educational Excellence First (Principle I)
- [x] Learning objectives: 3-5 measurable outcomes per chapter (Bloom's Taxonomy)
- [x] Word count: 800-1000 words per chapter (will be validated)
- [x] Structure: Intro → Learning Objectives → Hook → Concept → Mermaid Diagram → Code Example → Exercises → Summary → Preview Next Chapter (as specified)
- [x] Progressive complexity: Each chapter builds on previous (prerequisites clearly stated)
- [x] Real-world analogies for abstract concepts (required in spec)
- [x] All code examples will be complete, runnable, and tested
- [x] Step-by-step explanations accompanying each code block
- [x] Common errors section with troubleshooting tips
- [x] Practical exercises: 1-2 hands-on tasks per chapter
- [x] Beginner-friendly tone without oversimplification
- [x] No jargon without clear explanation

### Technical Accuracy & Verifiability (Principle II) - NON-NEGOTIABLE
- [x] ROS 2 commands, APIs, file paths will be exact and tested
- [x] Version-specific info clearly marked (ROS 2 Humble Hawksbill)
- [x] Terminology consistent with official ROS 2 documentation
- [x] Acronyms defined on first use (URDF - Unified Robot Description Format)
- [x] Links to official docs for deeper dives
- [x] All code examples validated for runnability before commit
- [x] Content will be reviewed for accuracy
- [x] Hardware requirements explicitly stated (NVIDIA RTX 4070 Ti or higher, Jetson Orin Nano)

### Spec-Driven Development (Principle III)
- [x] Feature workflow: spec → clarify → plan → tasks → implement → commit (following this process)
- [x] Constitution committed first before any features (already done)
- [x] All architectural decisions will be documented in ADRs
- [x] ADRs committed alongside relevant features

### Type Safety & Async-First Design (Principle IV)
- [x] TypeScript strict mode enabled for Docusaurus frontend
- [x] Python type hints required for content generation tools
- [x] All I/O operations async where appropriate

### Security & Privacy by Design (Principle V)
- [x] No API keys in code (environment variables only)
- [x] Better-auth sessions with httpOnly cookies, encrypted
- [x] Content validation: All user-generated content sanitized
- [x] No sensitive data in logs

### Testing & Validation (Principle VI)
- [x] Content validation: All code examples tested for runnability
- [x] Exercise validation: All exercises verified for educational value
- [x] Content structure validation: All chapters follow required format
- [x] Performance targets: Docusaurus build time < 2 minutes

### Progressive Enhancement & Graceful Degradation (Principle VII)
- [x] Core content accessible on all devices
- [x] TypeScript: React Error Boundaries for components
- [x] User-facing errors show friendly messages

### Performance & Scalability (Principle VIII)
- [x] Docusaurus build time: < 2 minutes
- [x] Mobile-responsive: All content readable on 375px viewport
- [x] Content loading: < 3 seconds for all textbook pages

### Observability & Debugging (Principle IX)
- [x] Structured logging in JSON format
- [x] Content validation logs for textbook quality assurance

### Simplicity & Pragmatism (Principle X)
- [x] Implement content as specified; avoid over-engineering
- [x] Content organization: Follow CHAPTERS_STRUCTURE.md exactly
- [x] Maintain clear pedagogical structure
- [x] Focus on educational value over technical complexity

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-textbook/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
physical-ai-textbook/
├── frontend/                    # Docusaurus Educational Platform
│   ├── docs/                    # Educational content (MDX)
│   │   ├── part-1-foundations-lab/           # Part 1: Foundations Lab (Chapters 1-3)
│   │   │   ├── chapter-01-embodied-ai.mdx
│   │   │   ├── chapter-02-hardware-setup.mdx
│   │   │   └── chapter-03-physical-ai-architecture.mdx
│   │   ├── part-2-robotic-nervous-system/    # Part 2: Robotic Nervous System (Chapters 4-9)
│   │   │   ├── chapter-04-ros2-architecture.mdx
│   │   │   ├── chapter-05-nodes-topics-services.mdx
│   │   │   ├── chapter-06-python-rclpy.mdx
│   │   │   ├── chapter-07-urdf-humanoids.mdx
│   │   │   ├── chapter-08-launch-parameters.mdx
│   │   │   └── chapter-09-first-ros2-package.mdx
│   │   ├── part-3-digital-twin/              # Part 3: Digital Twin (Chapters 10-15)
│   │   │   ├── chapter-10-physics-simulation-intro.mdx
│   │   │   ├── chapter-11-gazebo-setup.mdx
│   │   │   ├── chapter-12-urdf-sdf-formats.mdx
│   │   │   ├── chapter-13-sensor-simulation.mdx
│   │   │   ├── chapter-14-unity-visualization.mdx
│   │   │   └── chapter-15-realistic-environments.mdx
│   │   ├── part-4-ai-robot-brain/            # Part 4: AI-Robot Brain (Chapters 16-23)
│   │   │   ├── chapter-16-isaac-overview.mdx
│   │   │   ├── chapter-17-isaac-sim.mdx
│   │   │   ├── chapter-18-isaac-ros-vslam.mdx
│   │   │   ├── chapter-19-nav2-for-humanoid-navigation.mdx
│   │   │   ├── chapter-20-humanoid-kinematics.mdx
│   │   │   ├── chapter-21-bipedal-locomotion.mdx
│   │   │   ├── chapter-22-manipulation-grasping.mdx
│   │   │   └── chapter-23-sim-to-real.mdx
│   │   └── part-5-vla-capstone/              # Part 5: Vision-Language-Action Capstone (Chapters 24-28)
│   │       ├── chapter-24-vla-intro.mdx
│   │       ├── chapter-25-voice-to-action.mdx
│   │       ├── chapter-26-cognitive-planning.mdx
│   │       ├── chapter-27-multimodal-interaction.mdx
│   │       └── chapter-28-capstone-project.mdx
│   ├── src/
│   │   ├── components/          # Docusaurus components
│   │   │   ├── Auth/            # Authentication Components (Modal, Forms)
│   │   │   └── Learning/        # Educational components
│   │   ├── pages/               # Custom pages
│   │   ├── theme/               # Docusaurus theme customization
│   │   └── lib/                 # API clients (auth-client.ts)
│   ├── static/                  # Static assets
│   ├── docusaurus.config.ts     # Docusaurus configuration
│   ├── sidebars.ts              # Navigation structure
│   └── package.json             # Dependencies
│
├── content-tools/               # Python content generation tools
│   ├── scripts/                 # Content generation scripts
│   │   ├── generate_chapters.py # Chapter generation from templates
│   │   └── validate_content.py  # Content validation tools
│   ├── requirements.txt         # Python dependencies
│   └── pyproject.toml           # Project configuration
│
├── auth-service/                # Node.js Authentication Microservice
│   ├── src/
│   │   ├── auth.ts              # Better Auth Config
│   │   ├── db-setup.sql         # Database Schema
│   │   └── index.ts             # Express Server
│   ├── better-auth_migrations/  # SQL Migrations
│   ├── package.json
│   └── tsconfig.json
│
├── .claude/
│   ├── agents/                  # Claude agents for content generation
│   │   ├── physical-ai-author.md
│   │   ├── ros2-code-generator.md
│   │   └── book-architect.md
│   └── skills/                  # Claude skills for specific tasks
│       ├── docusaurus-style.md
│       ├── ros2-standards.md
│       ├── sim_to_real.md
│       ├── vla-patterns.md
│       ├── exercise-patterns.md
│       └── frontend-architect.md
└── specs/                       # Project Specifications
    └── 001-physical-ai-textbook/ # This specification
```

**Structure Decision**: Educational content platform with clear separation between content (MDX files in docs/), presentation (Docusaurus frontend), and supporting services (auth-service). The content follows the CHAPTERS_STRUCTURE.md with 5 parts and 28 chapters, each following the pedagogical template.

## Phase 0: Outline & Research

### Research Tasks Completed
- [x] Technology stack decisions (Docusaurus, Python tools, Better Auth)
- [x] Content structure and format research following CHAPTERS_STRUCTURE.md
- [x] Hardware requirements analysis
- [x] Implementation approach for tiered learning
- [x] Content generation strategy research

### Research Outcomes
- **research.md** created with detailed technology decisions and rationale
- All unknowns from Technical Context resolved
- Content organization patterns validated

## Phase 1: Design & Contracts

### Data Model Design
- [x] Entity definitions for Chapter, Part, User, Exercise, ContentChunk
- [x] Relationship mappings between entities
- [x] Validation rules for each entity
- [x] State transition definitions

### Content Structure Design
- [x] Chapter template definition (Intro → Learning Objectives → Hook → Concept → Mermaid Diagram → Code Example → Exercises → Summary → Preview Next Chapter)
- [x] Part organization following CHAPTERS_STRUCTURE.md
- [x] Pedagogical scaffolding elements defined
- [x] Hardware-tier content variations specified

### Documentation
- [x] **data-model.md** created with complete entity definitions
- [x] **quickstart.md** created with setup and development instructions
- [x] **content-template.md** created with chapter structure guidelines
- [x] Agent context updated with new technology stack

## Phase 2: Implementation Planning (tasks.md)

### Next Steps
The implementation plan will be detailed in **tasks.md** which will be generated using `/sp.tasks` command. The tasks will follow the 5-part structure from CHAPTERS_STRUCTURE.md:
1. Part 1: Foundations Lab (Chapters 1-3)
2. Part 2: Robotic Nervous System (Chapters 4-9)
3. Part 3: Digital Twin (Chapters 10-15)
4. Part 4: AI-Robot Brain (Chapters 16-23)
5. Part 5: Vision-Language-Action Capstone (Chapters 24-28)

Each chapter will follow the required structure: Intro → Learning Objectives → Hook → Concept → Mermaid Diagram → Code Example → Exercises → Summary → Preview Next Chapter

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |

## Re-evaluation of Constitution Check

After completing the design phase, all constitutional principles have been validated and incorporated:

### Educational Excellence First (Principle I)
- [x] Chapter structure follows required format (Intro → Learning Objectives → Hook → Concept → Mermaid Diagram → Code Example → Exercises → Summary → Preview Next Chapter)
- [x] Learning outcomes based on Bloom's Taxonomy included
- [x] Content validation processes defined

### Technical Accuracy & Verifiability (Principle II) - NON-NEGOTIABLE
- [x] Hardware requirements explicitly documented (RTX 4070 Ti+, Jetson Orin Nano)
- [x] ROS 2 Humble specific content validated
- [x] Code example validation processes defined

### Type Safety & Async-First Design (Principle IV)
- [x] TypeScript strict mode enforced in frontend
- [x] Python type hints required in content generation tools

### Security & Privacy by Design (Principle V)
- [x] Content validation: All user-generated content sanitized
- [x] Environment variable management for secrets

### Testing & Validation (Principle VI)
- [x] Content validation tests included
- [x] Exercise validation processes defined
- [x] Performance targets defined (Docusaurus build time < 2 minutes)

## Claude Agents & Skills Utilization

### Agents to be Used
- **physical-ai-author**: For generating educational content following pedagogical best practices
- **ros2-code-generator**: For creating production-grade ROS 2 Python code examples
- **book-architect**: For managing Docusaurus 3.x multi-file chapter structure following CHAPTERS_STRUCTURE.md
- **auth-engineer**: For implementing authentication and user management features
- **docusaurus-i18n-specialist**: For implementing multilingual support if needed

### Skills to be Used
- **docusaurus-style**: For writing and formatting MDX content with proper frontmatter
- **ros2-standards**: For authoritative guidelines on ROS 2 (Humble) Python code
- **sim_to_real**: For bridging simulated and real-world robotics deployment
- **vla-patterns**: For Vision-Language-Action model integration patterns
- **exercise-patterns**: For creating hands-on exercises in the textbook
- **frontend-architect**: For building the Docusaurus-based frontend with proper UI/UX

### Implementation Phases with Agents/Skills

**Phase 1: Content Creation**
- Use `physical-ai-author` agent to generate chapter content following educational standards
- Use `ros2-code-generator` agent to create accurate, runnable code examples
- Use `book-architect` agent to organize chapters into parts with proper navigation following CHAPTERS_STRUCTURE.md

**Phase 2: Platform Enhancement**
- Use `auth-engineer` agent to implement user authentication
- Use `frontend-architect` skill for proper Docusaurus customization

**Phase 3: Content Refinement**
- Use `docusaurus-style` skill for proper MDX formatting
- Use `exercise-patterns` skill for creating exercises
- Use `docusaurus-i18n-specialist` if multilingual support is needed

**Phase 4: Integration & Validation**
- Use `sim_to_real` skill for bridging simulation and real hardware content
- Use `vla-patterns` skill for advanced robotics concepts
- Validate all content against pedagogical standards
