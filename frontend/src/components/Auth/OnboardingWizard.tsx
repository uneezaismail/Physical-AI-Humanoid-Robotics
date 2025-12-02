/**
 * OnboardingWizard - Personalization Questionnaire
 * Theme: Cyber-Physical
 */

import React, { useState } from 'react';
import styles from './styles.module.css';

export interface UserProfile {
  experience: 'beginner' | 'intermediate' | 'expert';
  language: 'python' | 'cpp' | 'none';
  hardware: 'laptop' | 'workstation';
  os: 'windows' | 'linux' | 'macos';
  goal: 'academic' | 'builder';
}

interface OnboardingWizardProps {
  onComplete: (data: UserProfile) => void;
}

const questions = [
  {
    id: 'experience',
    text: "How would you rate your current experience with AI & Robotics?",
    options: [
      { value: 'beginner', label: 'Beginner', icon: '🌱', desc: "I'm new to this" },
      { value: 'intermediate', label: 'Intermediate', icon: '🌿', desc: "I understand the basics" },
      { value: 'expert', label: 'Expert', icon: '🌳', desc: "I work in this field" },
    ]
  },
  {
    id: 'language',
    text: "Which programming languages are you comfortable with?",
    options: [
      { value: 'python', label: 'Python', icon: '🐍', desc: "Course Default" },
      { value: 'cpp', label: 'C++', icon: '⚙️', desc: "I know this too" },
      { value: 'none', label: 'None', icon: '❌', desc: "Little experience" },
    ]
  },
  {
    id: 'hardware',
    text: "What kind of computer will you use for the simulations?",
    options: [
      { value: 'laptop', label: 'Standard Laptop', icon: '💻', desc: "No dedicated GPU / MacBook" },
      { value: 'workstation', label: 'Workstation', icon: '🎮', desc: "Has NVIDIA RTX GPU" },
    ]
  },
  {
    id: 'os',
    text: "What is your Operating System?",
    options: [
      { value: 'windows', label: 'Windows', icon: '🪟', desc: "WSL2 Required" },
      { value: 'linux', label: 'Linux', icon: '🐧', desc: "Ubuntu Native" },
      { value: 'macos', label: 'macOS', icon: '🍎', desc: "Docker Required" },
    ]
  },
  {
    id: 'goal',
    text: "What is your main goal for this book?",
    options: [
      { value: 'academic', label: 'Academic / Theory', icon: '🎓', desc: "I want to learn concepts" },
      { value: 'builder', label: 'Builder / Hobbyist', icon: '🛠️', desc: "I want to build a robot" },
    ]
  }
];

const OnboardingWizard: React.FC<OnboardingWizardProps> = ({ onComplete }) => {
  const [currentStep, setCurrentStep] = useState(0);
  const [profile, setProfile] = useState<Partial<UserProfile>>({});

  const handleSelect = (value: string) => {
    const currentQuestionId = questions[currentStep].id;
    const updatedProfile = { ...profile, [currentQuestionId]: value };
    setProfile(updatedProfile);

    if (currentStep < questions.length - 1) {
      setTimeout(() => setCurrentStep(currentStep + 1), 300); // Small delay for visual feedback
    } else {
      onComplete(updatedProfile as UserProfile);
    }
  };

  const currentQ = questions[currentStep];

  return (
    <div className={styles.wizardContainer}>
      <h2 className={styles.title}>Personalization</h2>
      <p className={styles.subtitle}>Step {currentStep + 1} of {questions.length}</p>

      <div className={styles.progressBar}>
        {questions.map((_, idx) => (
          <div 
            key={idx} 
            className={`${styles.progressStep} ${idx <= currentStep ? styles.active : ''}`}
          />
        ))}
      </div>

      <h3 style={{ textAlign: 'center', marginBottom: '1.5rem' }}>{currentQ.text}</h3>

      <div className={styles.cardGrid}>
        {currentQ.options.map((opt) => (
          <div
            key={opt.value}
            className={`${styles.optionCard} ${profile[currentQ.id as keyof UserProfile] === opt.value ? styles.selected : ''}`}
            onClick={() => handleSelect(opt.value)}
          >
            <div className={styles.cardIcon}>{opt.icon}</div>
            <div className={styles.cardTitle}>{opt.label}</div>
            <div className={styles.cardDesc}>{opt.desc}</div>
          </div>
        ))}
      </div>
    </div>
  );
};

export default OnboardingWizard;
