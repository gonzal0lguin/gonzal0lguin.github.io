// Field guide for the species planted on the hill, keyed by the scientific name used in
// src/data/inventory.json (the spreadsheet's `Nombre científico` column). The inventory is
// generated and holds only what was written down in the field; everything interpretive —
// what a species is, and why it went into this particular slope — lives here.
//
// Written from general references on the central-Chilean sclerophyll matorral, not from a
// survey of this hill, and pending review by the person in the family who actually studied
// botany. Where sources disagree (mature height above all, which depends enormously on
// water), the figure given is the range those references agree on.

export type Habit = "tree" | "shrub";

/** Where the species sits relative to this valley — the honest version, not the brochure. */
export type Origin =
  /** Grows only in Chile. */
  | "endemic"
  /** Native to Chile and to neighbouring countries. */
  | "native"
  /** Native to the dry Andes further north, long naturalized in the valley. */
  | "naturalized";

export interface Species {
  /** Prefix of the aluminium tag: QY000, LT004… */
  code: string;
  family: string;
  habit: Habit;
  origin: Origin;
  /** Mature height in metres, as a range. */
  height: [number, number];
  /** Months it flowers in central Chile, 1–12, for the flowering calendar. */
  flowering: number[];
  /** iNaturalist taxon id: photos, observations near the hill, and a map. */
  inaturalist: number;
  /** An openly licensed photo from iNaturalist, credited as its author asks. */
  photo: { url: string; author: string; license: string };
  /** Where to read more. Only pages that were checked to exist. */
  links: { chilebosque?: string; wikipedia?: { es?: string; en?: string } };
  es: { name: string; aka?: string; about: string; why: string };
  en: { name: string; aka?: string; about: string; why: string };
}

export const SPECIES: Record<string, Species> = {
  "Quillaja saponaria": {
    code: "QY",
    family: "Quillajaceae",
    habit: "tree",
    origin: "endemic",
    height: [10, 20],
    flowering: [11, 12, 1],
    inaturalist: 181934,
    photo: { url: "https://inaturalist-open-data.s3.amazonaws.com/photos/174183543/medium.jpg", author: "katrina0020", license: "CC BY-NC" },
    links: { chilebosque: "http://www.chilebosque.cl/flora/quillaja_saponaria.html", wikipedia: { es: "https://es.wikipedia.org/wiki/Quillaja_saponaria", en: "https://en.wikipedia.org/wiki/Quillaja_saponaria" } },
    es: {
      name: "Quillay",
      about:
        "El árbol insignia del bosque esclerófilo chileno y el más grande que plantamos: perenne, de copa redonda y corteza gris cargada de saponinas, que al molerla hace espuma y se usó como jabón y shampoo mucho antes de que existiera el shampoo. Hoy esa misma molécula se purifica como adyuvante de vacunas, así que un árbol del cerro de al lado terminó en la cadena de suministro farmacéutica mundial. Aguanta sequía extrema, florece en pleno verano y sus flores blancas son uno de los grandes pastoreos de las abejas de la zona central.",
      why: "Es la especie que define el techo del bosque que este cerro podría llegar a tener. Raíz profunda, tolerancia brutal a la sequía, y una vez establecido no necesita a nadie. Es la apuesta a 30 años: los quillayes son los que algún día van a dar sombra suficiente para que germine todo lo demás solo.",
    },
    en: {
      name: "Quillay",
      aka: "Soapbark tree",
      about:
        "The flagship tree of the Chilean sclerophyll forest and the largest thing we planted: evergreen, round-crowned, with grey bark loaded with saponins that foam when ground and were used as soap and shampoo long before shampoo existed. That same molecule is now purified as a vaccine adjuvant, so a tree from the hill next door ended up in the global pharmaceutical supply chain. It takes extreme drought, flowers in midsummer, and its white flowers are one of the great nectar flows for local bees.",
      why: "This is the species that defines the ceiling of the forest this hill could eventually have. Deep roots, brutal drought tolerance, and once established it needs nobody. It's the 30-year bet: the quillayes are the ones that will someday cast enough shade for everything else to germinate on its own.",
    },
  },

  "Lithraea caustica": {
    code: "LT",
    family: "Anacardiaceae",
    habit: "tree",
    origin: "endemic",
    height: [3, 8],
    flowering: [9, 10, 11],
    inaturalist: 543470,
    photo: { url: "https://inaturalist-open-data.s3.amazonaws.com/photos/234318175/medium.jpeg", author: "javichinga", license: "CC BY-NC" },
    links: { chilebosque: "http://www.chilebosque.cl/flora/lithraea_caustica.html", wikipedia: { es: "https://es.wikipedia.org/wiki/Lithraea_caustica", en: "https://en.wikipedia.org/wiki/Lithraea_caustica" } },
    es: {
      name: "Litre",
      about:
        "Endémico de Chile, perenne, de hoja gruesa y coriácea, y probablemente el árbol más duro de la zona central: coloniza las laderas norte, las más secas y expuestas, donde nada más prospera. Tiene mala fama merecida — a mucha gente le provoca dermatitis de contacto, y la creencia popular es que basta dormir bajo su sombra — pero es un fijador de laderas de primer nivel y sus frutos alimentan aves todo el otoño.",
      why: "Lo elegimos justamente por eso: es el que sobrevive donde el riego no llega. Su rol en el proyecto es de planta nodriza, dar sombra y materia orgánica para que después se puedan meter especies más delicadas bajo su copa. Es la segunda especie más plantada del inventario.",
    },
    en: {
      name: "Litre",
      about:
        "Endemic to Chile, evergreen, with thick leathery leaves, and probably the toughest tree in central Chile: it colonizes the north-facing slopes, the driest and most exposed, where nothing else gets going. Its bad reputation is earned — it gives many people contact dermatitis, and folk belief holds that sleeping in its shade is enough — but it's a first-rate slope stabilizer and its fruit feeds birds all autumn.",
      why: "We chose it for exactly that reason: it's the one that survives where the watering doesn't reach. Its role in the project is as a nurse plant, casting shade and dropping organic matter so that more delicate species can later go in under its canopy. It's the second most planted species in the inventory.",
    },
  },

  "Schinus polygama": {
    code: "HG",
    family: "Anacardiaceae",
    habit: "tree",
    origin: "native",
    height: [2, 5],
    flowering: [9, 10, 11],
    inaturalist: 1117857,
    photo: { url: "https://inaturalist-open-data.s3.amazonaws.com/photos/115719660/medium.jpg", author: "Claudio Maureira", license: "CC BY-NC-SA" },
    links: { wikipedia: { en: "https://en.wikipedia.org/wiki/Schinus_polygama" } },
    es: {
      name: "Huingán",
      about:
        "Arbolito o arbusto grande, espinoso y de ramas colgantes, que en el valle crece en cualquier borde de camino sin que nadie lo riegue. Da drupas pequeñas de color violeta oscuro que las aves se comen en masa y con las que se hacía chicha de huingán. Es dioico: hay pies macho y pies hembra, y sólo los segundos fructifican.",
      why: "Es la especie más plantada del inventario, y por tres razones: se establece rápido, es espinoso — lo que lo protege del ganado y, de paso, convierte a cada huingán en una jaula natural donde pueden germinar otras semillas a salvo — y su fruto es comida de aves, que son las que van a traer las semillas del resto del cerro gratis.",
    },
    en: {
      name: "Huingán",
      aka: "Chilean pepper tree",
      about:
        "A small tree or large shrub, spiny and with drooping branches, which in this valley grows along any roadside with nobody watering it. It bears small dark-purple drupes that birds eat in quantity and that were once fermented into chicha de huingán. It's dioecious: there are male and female plants, and only the latter fruit.",
      why: "It's the most planted species in the inventory, for three reasons: it establishes fast, it's spiny — which protects it from livestock and, incidentally, turns every huingán into a natural cage where other seeds can germinate in safety — and its fruit is bird food, and birds are what will bring in the seeds for the rest of the hill for free.",
    },
  },

  "Schinus areira": {
    code: "PM",
    family: "Anacardiaceae",
    habit: "tree",
    origin: "naturalized",
    height: [5, 15],
    flowering: [10, 11, 12],
    inaturalist: 486531,
    photo: { url: "https://inaturalist-open-data.s3.amazonaws.com/photos/186182408/medium.jpeg", author: "Graeme Rigg", license: "CC BY-NC" },
    links: { wikipedia: { es: "https://es.wikipedia.org/wiki/Schinus_areira", en: "https://en.wikipedia.org/wiki/Schinus_areira" } },
    es: {
      name: "Pimiento",
      aka: "Molle, pimiento boliviano",
      about:
        "El árbol de las plazas del valle: copa ancha y llorona, hoja fina perenne y racimos de granos rosados que son la pimienta rosa de la cocina. Es nativo de los Andes secos más al norte (Perú, Bolivia, el norte de Chile y Argentina) y está naturalizado en la zona central desde hace siglos, así que estrictamente no es parte del matorral original de este cerro.",
      why: "Es la única especie del inventario que no es del matorral local, y está ahí a propósito: da sombra densa mucho antes que cualquier nativa, aguanta sequía sin pestañear y es un árbol que la gente de la población reconoce y quiere. Sombra temprana junto al camino = un lugar donde pararse = gente que vuelve al cerro. Si eso resulta un problema más adelante, el pimiento es el primero que se puede discutir.",
    },
    en: {
      name: "Pimiento",
      aka: "Peruvian pepper tree, molle",
      about:
        "The tree of the valley's town squares: broad weeping crown, fine evergreen leaves, and clusters of pink berries that are the pink peppercorns of the spice rack. It's native to the dry Andes further north (Peru, Bolivia, northern Chile and Argentina) and has been naturalized in central Chile for centuries, so strictly speaking it isn't part of this hill's original matorral.",
      why: "It's the only species in the inventory that isn't from the local matorral, and it's there on purpose: it casts dense shade long before any native will, takes drought without blinking, and it's a tree the neighbourhood recognizes and likes. Early shade by the path = somewhere to stand = people who come back to the hill. If that turns out to be a problem later, the pimiento is the first one up for debate.",
    },
  },

  "Kageneckia oblonga": {
    code: "BE",
    family: "Rosaceae",
    habit: "tree",
    origin: "endemic",
    height: [3, 8],
    flowering: [9, 10, 11],
    inaturalist: 636304,
    photo: { url: "https://inaturalist-open-data.s3.amazonaws.com/photos/313393856/medium.jpg", author: "Nicolás Villaseca Merino", license: "CC BY-SA" },
    links: { chilebosque: "http://www.chilebosque.cl/flora/kageneckia_oblonga.html", wikipedia: { es: "https://es.wikipedia.org/wiki/Kageneckia_oblonga", en: "https://en.wikipedia.org/wiki/Kageneckia_oblonga" } },
    es: {
      name: "Bollén",
      about:
        "Endémico de Chile central y, sorprendentemente, un pariente del manzano y del rosal: es de la familia Rosaceae. Árbol pequeño de hoja coriácea con el borde aserrado y flores blancas de cinco pétalos en primavera. También es dioico. Vive en las laderas secas del bosque esclerófilo, casi siempre acompañando al quillay y al litre.",
      why: "Diversidad de familia botánica. Casi todo lo demás que plantamos es Anacardiaceae o Euphorbiaceae; el bollén mete una familia entera distinta al cerro, con otra flor, otro polinizador y otro calendario. Por ahora hay exactamente uno, y es el único Rosaceae del inventario.",
    },
    en: {
      name: "Bollén",
      about:
        "Endemic to central Chile and, surprisingly, a relative of the apple and the rose: it's in the Rosaceae. A small tree with leathery, saw-edged leaves and white five-petalled flowers in spring. Also dioecious. It lives on the dry slopes of the sclerophyll forest, almost always alongside quillay and litre.",
      why: "Botanical-family diversity. Nearly everything else we planted is Anacardiaceae or Euphorbiaceae; the bollén brings a whole different family onto the hill, with a different flower, a different pollinator and a different calendar. For now there is exactly one, and it's the only Rosaceae in the inventory.",
    },
  },

  "Geoffroea decorticans": {
    code: "CH",
    family: "Fabaceae",
    habit: "tree",
    origin: "native",
    height: [4, 10],
    flowering: [9, 10, 11],
    inaturalist: 139069,
    photo: { url: "https://inaturalist-open-data.s3.amazonaws.com/photos/329677092/medium.jpg", author: "Tomás Carranza Perales", license: "CC BY" },
    links: { chilebosque: "http://www.chilebosque.cl/flora/geoffroea_decorticans.html", wikipedia: { es: "https://es.wikipedia.org/wiki/Geoffroea_decorticans", en: "https://en.wikipedia.org/wiki/Geoffroea_decorticans" } },
    es: {
      name: "Chañar",
      about:
        "Una leguminosa espinosa y caducifolia de las zonas áridas del sur de Sudamérica, con una corteza verde brillante que se descascara en láminas (de ahí el *decorticans*) y que fotosintetiza aunque el árbol esté sin hojas. Da frutos dulces con los que se hace arrope de chañar. Raíces muy profundas y tolerancia a la sequía de otro nivel.",
      why: "Por ser caducifolio, que en un cerro como este es una ventaja y no un defecto: da sombra en el verano, cuando hace falta, y deja pasar la luz en invierno, cuando las hierbas y flores del suelo la necesitan para su temporada. También es la única leguminosa que plantamos, en un cerro cuyo árbol dominante — el espino — es precisamente otra leguminosa.",
    },
    en: {
      name: "Chañar",
      about:
        "A spiny, deciduous legume from the arid parts of southern South America, with bright green bark that peels off in sheets (hence *decorticans*) and photosynthesizes even when the tree has no leaves. It bears sweet fruit that gets boiled down into arrope de chañar. Very deep roots and drought tolerance on another level.",
      why: "Because it's deciduous, which on a hill like this is a feature and not a flaw: it shades the ground in summer, when that's what's needed, and lets the light through in winter, when the herbs and flowers on the ground need it for their season. It's also the only legume we planted, on a hill whose dominant tree — the espino — happens to be another legume.",
    },
  },

  "Colliguaja odorifera": {
    code: "CY",
    family: "Euphorbiaceae",
    habit: "shrub",
    origin: "endemic",
    height: [1, 2],
    flowering: [8, 9, 10],
    inaturalist: 487719,
    photo: { url: "https://inaturalist-open-data.s3.amazonaws.com/photos/26316527/medium.jpg", author: "Javier Conejeros Gastó", license: "CC BY" },
    links: { wikipedia: { es: "https://es.wikipedia.org/wiki/Colliguaja_odorifera", en: "https://en.wikipedia.org/wiki/Colliguaja_odorifera" } },
    es: {
      name: "Colliguay",
      about:
        "Arbusto endémico de la zona central, compacto y de un verde intenso, uno de los más abundantes del matorral. Como toda Euphorbiaceae, suelta un látex blanco al cortarlo que es cáustico e irritante. Rebrota desde la base después del fuego, lo que lo hace una de las especies que primero vuelven a aparecer después de un incendio.",
      why: "Es estructura del estrato arbustivo, que es la capa que más le falta a este cerro: hay espinos arriba y pasto abajo, y casi nada entremedio. Además el látex lo hace poco apetecible para animales, así que es una de las pocas que podemos plantar sin protección y esperar que siga ahí al mes siguiente.",
    },
    en: {
      name: "Colliguay",
      about:
        "A shrub endemic to central Chile, compact and deep green, one of the most abundant in the matorral. Like every Euphorbiaceae it bleeds a white latex when cut, caustic and irritant. It resprouts from the base after fire, which makes it one of the first species to reappear after a burn.",
      why: "It's structure for the shrub layer, which is the layer this hill is missing most: there are espinos above and grass below, and almost nothing in between. The latex also makes it unappetizing to animals, so it's one of the few we can plant unprotected and expect to still be there a month later.",
    },
  },

  "Lycium chilense": {
    code: "CO",
    family: "Solanaceae",
    habit: "shrub",
    origin: "native",
    height: [1, 2],
    flowering: [9, 10, 11, 12],
    inaturalist: 447537,
    photo: { url: "https://inaturalist-open-data.s3.amazonaws.com/photos/320538846/medium.jpg", author: "Eric Rojas-Abarca", license: "CC BY-NC" },
    links: { wikipedia: { es: "https://es.wikipedia.org/wiki/Lycium_chilense" } },
    es: {
      name: "Coralillo",
      aka: "Lycium",
      about:
        "Arbusto bajo, muy ramificado y armado de espinas, de la familia del tomate y la papa. Da bayas rojas pequeñas y brillantes — de ahí el nombre — que son comida de aves en pleno verano. Crece en suelos pobres, pedregosos y secos sin ninguna queja.",
      why: "Cobertura baja y espinosa, que es exactamente lo que se necesita en los bordes donde la gente y los perros entran al cerro: protege el suelo y marca el límite sin que haya que poner una reja. Y mete fruta carnosa al calendario en el mes más seco del año. (En el inventario aparece como «Coralillo», pero el nombre que vamos a usar de aquí en adelante es «Lycium».)",
    },
    en: {
      name: "Coralillo",
      aka: "Lycium",
      about:
        "A low, densely branched and thorny shrub, in the same family as tomato and potato. It bears small shiny red berries — hence the name — that are bird food in high summer. It grows in poor, stony, dry soil without complaint.",
      why: "Low thorny cover, which is exactly what's needed on the edges where people and dogs come onto the hill: it protects the soil and marks a limit without anyone having to put up a fence. And it puts fleshy fruit on the calendar in the driest month of the year. (The inventory still calls it «Coralillo», but the name we'll use from here on is «Lycium».)",
    },
  },

  "Sphaeralcea obtusiloba": {
    code: "MC",
    family: "Malvaceae",
    habit: "shrub",
    origin: "native",
    height: [0.4, 1],
    flowering: [10, 11, 12, 1, 2],
    inaturalist: 785518,
    photo: { url: "https://inaturalist-open-data.s3.amazonaws.com/photos/34332923/medium.jpeg", author: "mildred_ehrenfeld", license: "CC BY-NC" },
    links: {},
    es: {
      name: "Malva de cerro",
      about:
        "Un subarbusto de hoja gris afelpada que florece naranja-salmón durante media temporada, de octubre a febrero. Es de las primeras plantas en aparecer en suelo removido, taludes y orillas de camino, y una de las poquísimas nativas de la zona que tiene flor vistosa y larga.",
      why: "Polinizadores. El cerro tiene un peak de flores en septiembre–octubre (azulillos, alstroemerias) y después, nada: justo cuando más calor hace y más néctar se necesita. La malva de cerro cubre ese hueco del calendario. Y crece rápido sobre suelo desnudo, que es el 90 % de la ladera.",
    },
    en: {
      name: "Malva de cerro",
      aka: "Globemallow",
      about:
        "A subshrub with soft grey leaves that flowers salmon-orange for half the season, October to February. It's among the first plants to appear on disturbed soil, cut banks and roadsides, and one of the very few locally native plants with a showy, long-lasting flower.",
      why: "Pollinators. The hill has a flower peak in September–October (azulillos, alstroemerias) and then nothing: exactly when it's hottest and nectar is scarcest. The malva de cerro covers that gap in the calendar. And it grows fast on bare soil, which is 90 % of the slope.",
    },
  },

  "Baccharis linearis": {
    code: "RM",
    family: "Asteraceae",
    habit: "shrub",
    origin: "native",
    height: [1, 2],
    flowering: [2, 3, 4],
    inaturalist: 630561,
    photo: { url: "https://inaturalist-open-data.s3.amazonaws.com/photos/132709241/medium.jpg", author: "Carlos Gonzalez", license: "CC BY-NC" },
    links: { chilebosque: "http://www.chilebosque.cl/shrb/baline.html", wikipedia: { es: "https://es.wikipedia.org/wiki/Baccharis_linearis", en: "https://en.wikipedia.org/wiki/Baccharis_linearis" } },
    es: {
      name: "Romerillo",
      about:
        "El pionero por antonomasia: arbusto de hojas lineales y resinosas que es casi siempre la primera leñosa en colonizar un sitio degradado, un camino abandonado o un terreno quemado. Dioico, de flores blanquecinas muy visitadas por insectos, y florece a fines del verano, cuando ya no queda nada más floreciendo.",
      why: "Planta nodriza de manual. No lo plantamos por el romerillo mismo, sino porque sujeta el suelo, baja la temperatura superficial y crea el micrositio donde una semilla de quillay tiene alguna posibilidad. Es la especie «andamio» del proyecto, y en teoría la que sobra cuando el bosque ya está armado.",
    },
    en: {
      name: "Romerillo",
      about:
        "The pioneer par excellence: a shrub with narrow resinous leaves that is almost always the first woody plant to colonize a degraded site, an abandoned track or burnt ground. Dioecious, with whitish flowers heavily visited by insects, and it flowers at the end of summer, when nothing else is in bloom.",
      why: "A textbook nurse plant. We didn't plant it for itself, but because it holds soil, lowers the surface temperature and creates the microsite where a quillay seed has any chance at all. It's the project's scaffolding species, and in theory the one that becomes redundant once the forest is standing.",
    },
  },

  "Azara dentata": {
    code: "AZ",
    family: "Salicaceae",
    habit: "shrub",
    origin: "endemic",
    height: [2, 5],
    flowering: [9, 10, 11],
    inaturalist: 492086,
    photo: { url: "https://inaturalist-open-data.s3.amazonaws.com/photos/579127940/medium.jpg", author: "mbennette", license: "CC BY-NC" },
    links: { chilebosque: "http://www.chilebosque.cl/shrb/adent.html", wikipedia: { es: "https://es.wikipedia.org/wiki/Azara_dentata", en: "https://en.wikipedia.org/wiki/Azara_dentata" } },
    es: {
      name: "Azara",
      aka: "Corcolén",
      about:
        "Arbusto o arbolito endémico de hoja perenne con el borde dentado — de ahí el nombre — y flores amarillas en corimbo, sin pétalos pero con un penacho de estambres y un perfume dulce muy fuerte. Las aves comen su baya amarilla y dispersan la semilla; hay estudios en Chile central que muestran que el paso por el tracto digestivo de un ave mejora su germinación.",
      why: "Es la especie más exigente de agua del inventario, y es la razón concreta de que todo se haya plantado en la cara sur: en la ladera norte no tendría ninguna posibilidad. Si las azaras prenden, es la mejor señal de que esta cara del cerro realmente retiene humedad. Son nuestro experimento controlado, y a la vez lo que mejor va a oler en primavera.",
    },
    en: {
      name: "Azara",
      aka: "Corcolén, toothed azara",
      about:
        "An endemic evergreen shrub or small tree with toothed leaf margins — hence the name — and yellow flowers in corymbs: no petals, just a tuft of stamens and a very strong sweet scent. Birds eat its yellow berry and disperse the seed; studies in central Chile show that a pass through a bird's gut improves germination.",
      why: "It's the thirstiest species in the inventory, and the concrete reason everything went in on the south face: on the north slope it would have no chance at all. If the azaras take, that's the best evidence that this side of the hill really does hold moisture. They're our controlled experiment, and also the thing that will smell best in spring.",
    },
  },
};

/** Tag prefix → scientific name, for reading an ID like "LT004" back to a species. */
export const SPECIES_BY_CODE: Record<string, string> = Object.fromEntries(
  Object.entries(SPECIES).map(([scientific, species]) => [species.code, scientific]),
);
