import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'AI Robot Brain',
    description: 'Explore the principles of artificial intelligence applied to robotic systems.',
    to: '/docs/ai-robot-brain',
  },
  {
    title: 'Capstone Project',
    description: 'Complete projects that integrate all concepts learned throughout the textbook.',
    to: '/docs/capstone-project',
  },
  {
    title: 'Code Examples',
    description: 'Practical implementations and code snippets for real-world applications.',
    to: '/docs/code-examples',
  },
  {
    title: 'Digital Twin',
    description: 'Learn about virtual replicas of physical robotic systems for simulation.',
    to: '/docs/digital-twin',
  },
  {
    title: 'Educator Resources',
    description: 'Teaching materials and resources for educators and instructors.',
    to: '/docs/educator-resources',
  },
  {
    title: 'Robotic Nervous System',
    description: 'Understanding the control systems that enable humanoid robotics.',
    to: '/docs/robotic-nervous-system',
  },
];

function Feature({ title, description, to }) {
  return (
    <div className={clsx('col col--4', styles.featureCard)}>
      <Link to={to} className={styles.featureLink}>
        <div className={styles.featureItem}>
          <h3 className={styles.featureTitle}>{title}</h3>
          <p className={styles.featureDescription}>{description}</p>
          <div className={styles.featureArrow}>→</div>
        </div>
      </Link>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}