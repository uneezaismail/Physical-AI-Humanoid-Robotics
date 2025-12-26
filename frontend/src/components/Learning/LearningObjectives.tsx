import React from 'react';
import clsx from 'clsx';
import styles from './LearningObjectives.module.css';

interface LearningObjectivesProps {
  objectives: string[];
  bloomLevel?: 'remember' | 'understand' | 'apply' | 'analyze' | 'evaluate' | 'create';
}

const LearningObjectives: React.FC<LearningObjectivesProps> = ({
  objectives,
  bloomLevel = 'understand'
}) => {
  const bloomClass = styles[`bloom-${bloomLevel}`];

  return (
    <div className={clsx(styles.learningObjectives, bloomClass)}>
      <h3 className={styles.objectivesTitle}>Learning Objectives</h3>
      <p className={styles.bloomLevel}>Based on Bloom's Taxonomy: {bloomLevel.charAt(0).toUpperCase() + bloomLevel.slice(1)}</p>
      <ul className={styles.objectivesList}>
        {objectives.map((objective, index) => (
          <li key={index} className={styles.objectiveItem}>
            {objective}
          </li>
        ))}
      </ul>
    </div>
  );
};

export default LearningObjectives;