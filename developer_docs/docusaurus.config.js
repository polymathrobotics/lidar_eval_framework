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

  // Source repo: https://github.com/polymathrobotics/lidar_eval_framework
  // GitHub Pages therefore serves this site at:
  //     https://polymathrobotics.github.io/lidar_eval_framework/
  // Docusaurus wants that split into origin (`url`) and sub-path (`baseUrl`). A wrong
  // baseUrl is the usual cause of a deployed site loading with no CSS or JS.
  url: 'https://polymathrobotics.github.io',
  baseUrl: '/lidar_eval_framework/',

  // Same repo again, as the fields `docusaurus deploy` and the "Edit this page"
  // links need it in.
  organizationName: 'polymathrobotics',
  projectName: 'lidar_eval_framework',
  deploymentBranch: 'gh-pages',

  // GitHub Pages treats /docs/intro and /docs/intro/ as different URLs; without this
  // some links 404 depending on how they were written.
  trailingSlash: false,

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
          // "Edit this page" links. The docs live in developer_docs/ inside the repo,
          // so that prefix is part of the path.
          editUrl:
            'https://github.com/polymathrobotics/lidar_eval_framework/tree/main/developer_docs/',
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
            href: 'https://github.com/polymathrobotics/lidar_eval_framework',
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
                href: 'https://github.com/polymathrobotics/lidar_eval_framework',
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
