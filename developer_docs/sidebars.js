// @ts-check

/**
 * @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  devSidebar: [
    {
      type: 'doc',
      id: 'intro', // Points perfectly to your docs/intro.md file
      label: '📦 1. Overview/Getting Started',
    },
    {
      type: 'doc',
      id: 'architecture', // Points perfectly to your docs/architecture.md file
      label: '🏗️ 2. Architecture / Core Concepts',
    },
    {
      type: 'doc',
      id: 'developer-guide', // Points perfectly to your docs/developer-guide.md file
      label: '🛠️ 3. Developer Guide',
    },
    {
      type: 'doc',
      id: 'contributing', // Points perfectly to your docs/contributing.md file
      label: '🤝 4. Contribute',
    },
  ],
};

export default sidebars;
