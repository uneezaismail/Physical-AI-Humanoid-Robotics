import type { ReactNode } from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type HighlightItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
  animationClass: string;
};

const HighlightList: HighlightItem[] = [
  {
    title: 'ROS 2 Fundamentals',
    // Using React SVG as placeholder for ROS 2
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    animationClass: styles.spinSlow,
    description: (
      <>
        Master the <strong>Robotic Operating System</strong> that powers modern robots. 
        Learn nodes, topics, services, and advanced patterns for building intelligent robotic systems.
      </>
    ),
  },
  {
    title: 'Robot Simulation',
    // Using Tree SVG as placeholder for Environment/Gazebo
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    animationClass: styles.float,
    description: (
      <>
        Build and test robots in <strong>Gazebo</strong> before deploying to real hardware. 
        Create URDF models, simulate physics, and integrate sensors in a safe virtual environment.
      </>
    ),
  },
  {
    title: 'AI-Powered Intelligence',
    // Using Mountain SVG as placeholder for "High Level" AI
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    animationClass: styles.pulse,
    description: (
      <>
        Leverage <strong>NVIDIA Isaac</strong> and Vision-Language-Action models for perception, 
        navigation, and manipulation. Deploy AI to Jetson edge devices for autonomous operation.
      </>
    ),
  },
];

function Highlight({title, Svg, description, animationClass}: HighlightItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={clsx(styles.highlightSvg, animationClass)} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3" className={styles.highlightTitle}>{title}</Heading>
        <p className={styles.highlightDescription}>{description}</p>
      </div>
    </div>
  );
}

export default function BookHighlights(): ReactNode {
  return (
    <section className={styles.highlights}>
      <div className="container">
        <div className="row">
          {HighlightList.map((props, idx) => (
            <Highlight key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
