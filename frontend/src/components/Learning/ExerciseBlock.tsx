import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './ExerciseBlock.module.css';

interface ExerciseBlockProps {
  title?: string;
  difficulty?: 'beginner' | 'intermediate' | 'advanced';
  children: React.ReactNode;
  hint?: string;
  solution?: string;
}

const ExerciseBlock: React.FC<ExerciseBlockProps> = ({
  title = 'Exercise',
  difficulty = 'intermediate',
  children,
  hint,
  solution,
}) => {
  const [showHint, setShowHint] = useState(false);
  const [showSolution, setShowSolution] = useState(false);

  const difficultyClass = styles[`difficulty-${difficulty}`];

  return (
    <div className={clsx(styles.exerciseBlock, difficultyClass)}>
      <div className={styles.exerciseHeader}>
        <h3 className={styles.exerciseTitle}>{title}</h3>
        <span className={styles.difficultyBadge}>{difficulty.charAt(0).toUpperCase() + difficulty.slice(1)}</span>
      </div>

      <div className={styles.exerciseContent}>
        {children}
      </div>

      {hint && (
        <div className={styles.hintSection}>
          <button
            className={styles.hintButton}
            onClick={() => setShowHint(!showHint)}
            aria-expanded={showHint}
          >
            {showHint ? 'Hide Hint' : 'Show Hint'}
          </button>

          {showHint && (
            <div className={styles.hintContent}>
              <strong>Hint:</strong> {hint}
            </div>
          )}
        </div>
      )}

      {solution && (
        <div className={styles.solutionSection}>
          <button
            className={styles.solutionButton}
            onClick={() => setShowSolution(!showSolution)}
            aria-expanded={showSolution}
          >
            {showSolution ? 'Hide Solution' : 'Show Solution'}
          </button>

          {showSolution && (
            <div className={styles.solutionContent}>
              <strong>Solution:</strong>
              <div className={styles.solutionBody}>
                {solution}
              </div>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default ExerciseBlock;