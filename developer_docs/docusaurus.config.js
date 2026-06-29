// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'LiDAR Eval Framework',
  tagline: 'High-performance 3D point cloud benchmarking engine',
  favicon: 'img/favicon.ico',

  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  url: 'https://your-docusaurus-site.example.com',
  baseUrl: '/',

  // Fallback configuration error handling
  onBrokenLinks: 'warn', // Prevents build crashes on broken doc links during early setup

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          routeBasePath: 'docs', // Keeps path structures unified as /docs/...
        },
        blog: false, // DISABLED: Removes the default blog module completely
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      colorMode: {
        defaultMode: 'light',
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'LiDAR Eval Framework', // This overrides the top left navbar text
        logo: {
          alt: 'LiDAR Eval Framework Logo',
          src: 'img/logo.svg', // Swap this in static/img/ when you have a custom asset
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'devSidebar', // Links explicitly to the key inside sidebars.js
            position: 'left',
            label: 'Developer Docs',
          },
          {
            href: 'https://github.com/your-org/lidar-eval-framework',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Documentation',
            items: [
              {
                label: 'Overview & Onboarding',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Community & Ecosystem',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/',
              },
            ],
          },
          {
            title: 'Repository Source',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-org/lidar-eval-framework',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} LiDAR Evaluation Framework Portal. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        // Common language highlighters for a mechatronics/robotics portal
        additionalLanguages: ['bash', 'yaml', 'json', 'cpp', 'python'],
      },
    }),
};

export default config;
