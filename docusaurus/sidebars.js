// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Robotic Nervous System',
      items: [
        'robotic-nervous-system/index',
        'robotic-nervous-system/communication-architecture',
        'robotic-nervous-system/programming-integration',
        'robotic-nervous-system/robot-modeling'
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin',
      items: [
        'digital-twin/index',
        'digital-twin/physics-simulation',
        'digital-twin/sensor-simulation',
        'digital-twin/visualization'
      ],
    },
    {
      type: 'category',
      label: 'AI-Robot Brain',
      items: [
        'ai-robot-brain/index',
        'ai-robot-brain/perception-navigation',
        'ai-robot-brain/path-planning',
        'ai-robot-brain/sim-to-real-transfer'
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone-project/index',
        'capstone-project/voice-processing',
        'capstone-project/object-manipulation',
        'capstone-project/integration',
        'capstone-project/implementation',
        'capstone-project/validation'
      ],
    },
    {
      type: 'category',
      label: 'Educator Resources',
      items: [
        'educator-resources/index',
        'educator-resources/curriculum-guides',
        'educator-resources/solution-manuals',
        'educator-resources/assessment-tools'
      ],
    },
    {
      type: 'category',
      label: 'Code Examples',
      items: [
        'code-examples/index',
        'code-examples/ros2',
        'code-examples/gazebo',
        'code-examples/isaac',
        'code-examples/python',
        'code-examples/vla'
      ],
    },
  ],
};

module.exports = sidebars;