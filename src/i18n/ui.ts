// UI strings and URL helpers for the two site languages.
// Spanish is the default (unprefixed URLs); English lives under /en/.

export const languages = {
  es: { label: "Español", flag: "/flags/cl.svg", locale: "es-CL" },
  en: { label: "English", flag: "/flags/gb.svg", locale: "en-GB" },
} as const;

export type Lang = keyof typeof languages;
export const defaultLang: Lang = "es";

// `[...lang]` route param values: undefined → Spanish at the root, "en" → /en/...
export const langParams = [undefined, "en"] as const;

export function getLang(param?: string): Lang {
  return param === "en" ? "en" : "es";
}

export function getLangFromUrl(url: URL): Lang {
  return /^\/en(\/|$)/.test(url.pathname) ? "en" : "es";
}

/** Remove the language prefix from a pathname: "/en/cv/" → "/cv/". */
export function stripLang(pathname: string): string {
  return pathname.replace(/^\/en(?=\/|$)/, "") || "/";
}

/** Build a path for `lang` from a language-neutral path such as "/cv". */
export function localizePath(lang: Lang, path: string): string {
  if (lang === defaultLang) return path;
  return path === "/" ? "/en/" : `/en${path}`;
}

export function formatDate(date: Date, lang: Lang): string {
  // Dates in frontmatter are calendar days; format in UTC so they don't shift a day
  // depending on the timezone of the machine running the build.
  return new Intl.DateTimeFormat(languages[lang].locale, { dateStyle: "medium", timeZone: "UTC" }).format(date);
}

const ui = {
  es: {
    "site.title": "Gonzalo Olguín | Portafolio y Montañismo",
    "site.description":
      "Portafolio personal y bitácora de montaña de Gonzalo Olguín. Proyectos de robótica e IA, y aventuras en la montaña.",

    "nav.home": "Inicio",
    "nav.cv": "CV",
    "nav.projects": "Proyectos",
    "nav.mountaineering": "Montañismo",
    "nav.news": "Noticias",
    "nav.contact": "Contacto",
    "nav.openMenu": "Abrir menú",

    "controls.language": "Idioma",
    "controls.theme": "Cambiar entre tema claro y oscuro",

    "home.connect": "¡Conversemos!",
    "home.viewCv": "Ver mi CV",
    "home.latestNews": "Últimas noticias",
    "home.latestProjects": "Últimos proyectos",
    "home.latestAscents": "Últimas salidas a la montaña",

    "cv.title": "Currículum Vitae",
    "cv.download": "Descargar CV en PDF",

    "projects.title": "Proyectos",
    "projects.intro":
      "¡Bienvenido a mi portafolio de proyectos! Aquí encontrarás bitácoras detalladas de todo lo que he construido, investigado o con lo que he cacharreado. Para mantener el orden, los dividí en categorías. Elige una categoría para ver sus proyectos.",
    "projects.empty": "Todavía no hay proyectos en esta categoría. ¡Vuelve pronto!",
    "projects.cat.academic-projects": "Proyectos académicos",
    "projects.cat.academic-projects.desc": "Proyectos desarrollados en mis cursos universitarios y en investigación.",
    "projects.cat.hobby-projects": "Proyectos hobby",
    "projects.cat.hobby-projects.desc": "Proyectos paralelos entretenidos, robótica y experimentos de ingeniería.",
    "projects.cat.personal": "Personal",
    "projects.cat.personal.desc": "Carpintería, DIY y otras creaciones personales.",

    "ascents.title": "Bitácora de Montañismo",
    "ascents.intro":
      "¡Bienvenido a mi bitácora de montaña! Aquí documento mis expediciones, travesías y cumbres. Abajo puedes ver un resumen de mis estadísticas y un mapa interactivo con los lugares que he visitado.",
    "ascents.all": "Todas las salidas",
    "ascents.statSummits": "Cumbres",
    "ascents.statSummitsDesc": "Entradas en la bitácora",
    "ascents.statVertical": "Desnivel acumulado",
    "ascents.statDistance": "Distancia total",
    "ascents.viewEntry": "Ver salida",
    "ascents.mountain": "Montaña",
    "ascents.elevation": "Altitud",
    "ascents.date": "Fecha",

    "news.title": "Noticias y novedades",
    "news.empty": "Todavía no hay noticias. ¡Vuelve pronto!",

    "tag.title": "Etiqueta",
    "list.empty": "No hay nada que mostrar por ahora. ¡Vuelve pronto!",
    "list.sorry": "¡Lo siento!",
    "list.newer": "Más recientes",
    "list.older": "Más antiguos",

    "post.updated": "Actualizado el",
    "post.untranslated": "Esta entrada aún no está traducida al inglés.",

    "route.kicker": "Ruta GPS",
    "route.distance": "Distancia",
    "route.gain": "Desnivel positivo",
    "route.loss": "Desnivel negativo",
    "route.maxEle": "Altitud máxima",
    "route.minEle": "Altitud mínima",
    "route.moving": "En movimiento",
    "route.total": "Tiempo total",
    "route.heartRate": "Frecuencia cardíaca",
    "route.heartRateValue": "{avg} media · {max} máx",
    "route.maxGrade": "Pendiente máx.",
    "route.estimate": "Estimación DIN 33466",
    "route.estimateHint": "Tiempo de marcha según la norma DIN 33466 que usa el Club Alpino Alemán (DAV): 4 km/h en plano, 300 m/h de subida y 500 m/h de bajada.",
    "route.shape.loop": "Circuito",
    "route.shape.out-and-back": "Ida y vuelta",
    "route.shape.one-way": "Solo ida",
    "route.days": "{n} días",
    "route.day": "Día {n}",
    "route.camp": "Campamento",
    "route.campRest": "{time} de descanso",
    "route.summit": "Cumbre",
    "route.start": "Inicio",
    "route.finish": "Fin",
    "route.startFinish": "Inicio y fin",
    "route.colorBy": "Colorear por",
    "route.mode.grade": "Pendiente",
    "route.mode.elevation": "Altitud",
    "route.mode.hr": "Pulso",
    "route.profile": "Perfil de elevación",
    "route.play": "Recorrer la ruta",
    "route.pause": "Pausar",
    "route.recenter": "Centrar la ruta",
    "route.fullscreen": "Pantalla completa",
    "route.download": "Descargar GPX",
    "route.layer.topo": "Topográfico",
    "route.layer.satellite": "Satélite",
    "route.layer.streets": "Calles",
    "route.zoomHint": "Haz clic en el mapa para hacer zoom con la rueda",
    "route.bpm": "lpm",
    "ascents.statRoutes": "En {n} rutas con GPS",
    "ascents.statRoute": "En 1 ruta con GPS",
  },
  en: {
    "site.title": "Gonzalo Olguín | Portfolio & Mountaineering",
    "site.description":
      "Personal portfolio and mountaineering log of Gonzalo Olguín. Explore projects in robotics, AI, and mountain adventures.",

    "nav.home": "Home",
    "nav.cv": "CV",
    "nav.projects": "Projects",
    "nav.mountaineering": "Mountaineering",
    "nav.news": "News",
    "nav.contact": "Contact",
    "nav.openMenu": "Open menu",

    "controls.language": "Language",
    "controls.theme": "Toggle light and dark theme",

    "home.connect": "Let's connect!",
    "home.viewCv": "View my CV",
    "home.latestNews": "Latest News",
    "home.latestProjects": "Latest Projects",
    "home.latestAscents": "Latest Mountain Trips",

    "cv.title": "Curriculum Vitae",
    "cv.download": "Download CV as PDF",

    "projects.title": "Projects",
    "projects.intro":
      "Welcome to my project portfolio! Here you can find detailed logs for all the things I've built, researched, or tinkered with. To keep things organized, I've split them into categories. Choose a category below to see the related projects.",
    "projects.empty": "There are no projects to show in this category yet. Check back later!",
    "projects.cat.academic-projects": "Academic projects",
    "projects.cat.academic-projects.desc": "Projects developed during my university courses and research.",
    "projects.cat.hobby-projects": "Hobby projects",
    "projects.cat.hobby-projects.desc": "Fun side-projects, robotics, and engineering experiments.",
    "projects.cat.personal": "Personal",
    "projects.cat.personal.desc": "Woodworking, DIY, and other personal creations.",

    "ascents.title": "Mountaineering Log",
    "ascents.intro":
      "Welcome to my mountaineering log! This is where I document my expeditions, traverses, and summits. Below you can see a summary of my mountaineering stats and an interactive map showing the places I've visited.",
    "ascents.all": "All trips",
    "ascents.statSummits": "Summits",
    "ascents.statSummitsDesc": "Total log entries",
    "ascents.statVertical": "Total elevation gain",
    "ascents.statDistance": "Total distance",
    "ascents.viewEntry": "View trip",
    "ascents.mountain": "Mountain",
    "ascents.elevation": "Elevation",
    "ascents.date": "Date",

    "news.title": "News & Updates",
    "news.empty": "There is no news to show at the moment. Check back later!",

    "tag.title": "Tag",
    "list.empty": "Nothing to show at the moment. Check back later!",
    "list.sorry": "Sorry!",
    "list.newer": "Newer",
    "list.older": "Older",

    "post.updated": "Last updated on",
    "post.untranslated": "This entry hasn't been translated into English yet, so it's shown in Spanish.",

    "route.kicker": "GPS route",
    "route.distance": "Distance",
    "route.gain": "Elevation gain",
    "route.loss": "Elevation loss",
    "route.maxEle": "Highest point",
    "route.minEle": "Lowest point",
    "route.moving": "Moving time",
    "route.total": "Total time",
    "route.heartRate": "Heart rate",
    "route.heartRateValue": "{avg} avg · {max} max",
    "route.maxGrade": "Steepest grade",
    "route.estimate": "DIN 33466 estimate",
    "route.estimateHint": "Hiking time following the DIN 33466 standard used by the German Alpine Club (DAV): 4 km/h on the flat, 300 m/h uphill and 500 m/h downhill.",
    "route.shape.loop": "Loop",
    "route.shape.out-and-back": "Out and back",
    "route.shape.one-way": "One way",
    "route.days": "{n} days",
    "route.day": "Day {n}",
    "route.camp": "Camp",
    "route.campRest": "{time} of rest",
    "route.summit": "Summit",
    "route.start": "Start",
    "route.finish": "Finish",
    "route.startFinish": "Start & finish",
    "route.colorBy": "Color by",
    "route.mode.grade": "Grade",
    "route.mode.elevation": "Elevation",
    "route.mode.hr": "Heart rate",
    "route.profile": "Elevation profile",
    "route.play": "Replay the route",
    "route.pause": "Pause",
    "route.recenter": "Recenter the route",
    "route.fullscreen": "Fullscreen",
    "route.download": "Download GPX",
    "route.layer.topo": "Topographic",
    "route.layer.satellite": "Satellite",
    "route.layer.streets": "Streets",
    "route.zoomHint": "Click the map to zoom with the scroll wheel",
    "route.bpm": "bpm",
    "ascents.statRoutes": "Across {n} GPS routes",
    "ascents.statRoute": "From 1 GPS route",
  },
} as const;

export type UIKey = keyof (typeof ui)["es"];

export function useTranslations(lang: Lang) {
  // Optional {placeholders} are filled from `vars`: t("route.day", { n: 2 }) → "Día 2"
  return (key: UIKey, vars?: Record<string, string | number>): string => {
    const text: string = ui[lang][key] ?? ui[defaultLang][key];
    return vars ? text.replace(/\{(\w+)\}/g, (match, name) => String(vars[name] ?? match)) : text;
  };
}
