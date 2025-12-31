import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Physical AI',
    description: (
      <>
        Explore the principles of artificial intelligence applied to physical systems and robotics.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics',
    description: (
      <>
        Learn about the design, control, and implementation of humanoid robotic systems.
      </>
    ),
  },
  {
    title: 'Textbook Approach',
    description: (
      <>
        Comprehensive coverage from fundamentals to advanced topics in AI and robotics.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
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