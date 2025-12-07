// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro/intro',
    {
      type: 'category',
      label: 'Physical AI',
      items: ['physical-ai/intro', 'physical-ai/basics'],
    },
    {
      type: 'category',
      label: 'Humanoid Robotics',
      items: ['humanoid-robotics/intro', 'humanoid-robotics/design'],
    },
    {
      type: 'category',
      label: 'Integration',
      items: ['integration/applications', 'integration/future'],
    },
  ],
};

module.exports = sidebars;