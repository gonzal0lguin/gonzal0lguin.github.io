# Gonzalo Olguín | Portfolio & Ascents

Welcome to the source code for my personal portfolio, robotics/AI projects showcase, and mountain ascents blog. This website is built with [Astro](https://astro.build), [TailwindCSS](https://tailwindcss.com/), and [DaisyUI](https://daisyui.com/).

## Live Site
Visit the live site: [gonzal0lguin.github.io](https://gonzal0lguin.github.io)

## Local Development Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/gonzal0lguin/gonzal0lguin.github.io.git
   cd gonzal0lguin.github.io
   ```

2. **Install dependencies:**
   This project uses `pnpm` as the package manager. If you don't have it installed, you can install it via npm: `npm install -g pnpm`.
   ```bash
   pnpm install
   ```

3. **Start the development server:**
   ```bash
   pnpm run dev
   ```
   The site will be available at `http://localhost:4321`.

4. **Build for production:**
   ```bash
   pnpm run build
   ```

## Managing Content

This site uses Astro's Content Collections to manage structured markdown content. There are three main collections: **Projects**, **Ascents**, and **Blog**.

### 1. Projects
Located in `src/content/projects/`. 
To add a new project, create a Markdown (`.md`) file in this folder.

**Frontmatter format:**
```yaml
---
title: "Project Title"
description: "A short description of the project"
pubDate: "2024-01-27"
heroImage: "/assets/img/headers/project-image.png"
badge: "Featured"        # Optional badge to display on the card
category: "Robotics"     # Optional category
tags: ["ros2", "slam"]   # Optional tags
---
```
Write your project details using standard Markdown below the frontmatter.

### 2. Ascents
Located in `src/content/ascents/`.
This collection is specialized for mountain ascents and tracks specific mountain metadata.

**Frontmatter format:**
```yaml
---
title: "La Cruz"
description: "Ascent description"
pubDate: "2024-03-01"
mountain: "Cerro La Cruz"  # Optional
elevation: 2552            # Optional (in meters)
latitude: -33.435          # Optional
longitude: -70.470         # Optional
heroImage: "/assets/img/headers/mountain-image.jpg"
badge: "Solo"
tags: ["andes", "day-hike"]
---
```

### 3. Blog
Located in `src/content/blog/`.
Standard blog posts.

**Frontmatter format:**
```yaml
---
title: "Blog Post Title"
description: "Blog description"
pubDate: "2024-01-01"
heroImage: "/assets/img/headers/blog-header.jpg"
tags: ["personal", "update"]
---
```

## Adding Images

Store your images in the `public/assets/img/` directory.
- Headers/Hero images generally go in `public/assets/img/headers/`
- Post-specific images can go in `public/assets/img/posts/<post-name>/`

When referencing an image in your markdown or frontmatter, use the absolute path starting from the public root, e.g., `/assets/img/headers/image.jpg`.

## Site Configuration

Global site configuration is managed in two places:

1. **`src/config.ts`**: Contains the main `SITE_TITLE` and `SITE_DESCRIPTION` variables used for Open Graph tags and SEO.
2. **`astro.config.mjs`**: Contains the `site` URL and integrations (Tailwind, MDX, Math rendering plugins).

## UI & Theming

- The sidebar, header, and footer components are located in `src/components/`. 
- To edit links in the sidebar, modify `src/components/SideBarMenu.astro`.
- To change the site's colors or theme, edit the `data-theme` attribute on the `<html>` tag in `src/layouts/BaseLayout.astro`. This project uses DaisyUI, which provides many built-in themes.

## Customizing the Favicon

The favicon is located at `public/favicon.svg`. To change it, simply replace this file with a new SVG image. You can use icons from collections like [Lucide](https://lucide.dev/) or [Heroicons](https://heroicons.com/).

## Tech Stack
- [Astro](https://astro.build/) - Web framework
- [TailwindCSS](https://tailwindcss.com/) - Utility-first CSS
- [DaisyUI](https://daisyui.com/) - Tailwind CSS components
- [Remark Math](https://github.com/remarkjs/remark-math) / [Rehype Katex](https://github.com/remarkjs/remark-math/tree/main/packages/rehype-katex) - LaTeX rendering support
