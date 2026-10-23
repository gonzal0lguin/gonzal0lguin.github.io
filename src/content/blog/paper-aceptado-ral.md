---
title: "Paper aceptado en IEEE RA-L"
description: "Sem-NaVAE, el trabajo que salió de mi tesis de magíster, fue aceptado en IEEE Robotics and Automation Letters."
pubDate: "2026-06-03"
badge: "Paper"
tags: ["Paper", "Robotics"]
---

**Sem-NaVAE: Semantically-Guided Outdoor Mapless Navigation via Generative Trajectory Priors** fue aceptado para publicación en *IEEE Robotics and Automation Letters* (RA-L). Ya está online en IEEE Xplore: vol. 11, no. 8, pp. 9335–9342, agosto 2026.

El trabajo es con mi profesor guía, Javier Ruiz-del-Solar, y es el resultado directo de mi [tesis de magíster](/projects/tesis-navegacion-multinivel).

En corto: un robot navegando en exterior **sin ningún mapa**. Un CVAE genera más de 200 hipótesis de trayectoria por paso, y un modelo de visión-lenguaje liviano (CLIPSeg) las puntúa con segmentación de vocabulario abierto, usando preferencias escritas en lenguaje natural. En pruebas reales en rutas de 120 a 240 m en un sector nunca visto, el sistema alcanza **90% de tasa de éxito**: 10 puntos sobre el *baseline* más cercano sin mapa, y a 7 puntos de Nav2, que sí tiene mapa.

- 📄 [arXiv:2602.01429](https://arxiv.org/abs/2602.01429)
- 🎥 [Video de un run completo](https://youtu.be/i3R5ey5O2yk)
- 🧵 **[El post detallado sobre el paper está acá](/projects/sem-navae)** — métricas, *ablations*, los *baselines* y la prueba de largo alcance (>1 km) en un parque.
- 📚 Y si quieres la versión larga, con el dataset y todas las derivaciones, está el [post de la tesis completa](/projects/tesis-navegacion-multinivel).

Financiado parcialmente por FONDECYT 1251823 y ANID CIA250010.
