---
title: "Route analyzer: a GPX track tool for the mountains"
category: "hobby-projects"
description: "A free tool to view, join, mark and analyze your GPX files: map, profile, 3D view, climbs, splits and a time planner. All in your browser."
pubDate: 2026-10-19 19:00:00 -03
author: Gonzalo Olguin
tags: [montaña, gpx, mapas, herramientas]
heroImage: /assets/img/headers/route-tool-en.jpg
---

After every trip to the mountains I ended up with the same problem: the track was on my watch or on Wikiloc, split into two or three files because the recording stopped, and the apps I use only show the basics (distance, elevation gain and a line on a map). I wanted something that answered the questions you actually ask yourself in the mountains: *how long did the final climb take? how fast do I climb with a pack? what time do I need to leave to get back in daylight?*

That's how the **[Route analyzer](/en/ascents/route-tool)** came about. It's the same tool that draws the maps in my mountaineering log, now open for anyone to use with their own files. It's free, needs no account, and **your files never leave your browser**: everything is processed on your own computer or phone.

## How to use it

### 1. Upload one or more GPX files

Drop your `.gpx` files on the upload area (or click to pick them). It works with what Garmin, Coros, Suunto, Strava, Wikiloc, Komoot and almost any app exports.

If you recorded the route in several parts, upload them all together: the tool **orders them by the time they were recorded**, whatever their names, and joins them into a single route. In the example, the Punta Canaleta files have names that don't match the real order, and they still come out right: the first day's approach, the summit push from camp, and the descent.

![Upload area with three GPX files in chronological order](route-tool/upload-en.jpg "Three files from the same trip, joined in chronological order")

You can add or remove files at any time and the route is recalculated right away.

### 2. Read the route

The main figures are at the top: distance, elevation gain, highest point and moving time. Below them, the map and the elevation profile are linked: hovering over one moves a dot on the other, with the distance, elevation, grade, time of day and heart rate at that point.

![Route card with stats, map colored by grade and elevation profile](route-tool/card-en.jpg "The map colored by grade, with the camp and summit detected automatically")

A few things worth knowing:

- **Color by**: in the map legend you can switch between grade, elevation, speed, vertical speed (climbing in red, descending in blue) and heart rate, if your watch recorded it.
- **Days and camps**: any pause longer than 3 hours counts as a night. The tool splits the route into days and marks where you camped.
- **The ▶︎ button** replays the route from start to finish.
- **Layers**: topographic, satellite or streets, from the layers button.
- Click any point on the route to see its coordinates and open it in **Google Earth 3D** or **Google Maps 3D**.

### 3. Analyze a section

Drag the mouse (or your finger) across the profile to select a section. You get its distance, elevation gain and loss, average grade, time, vertical speed and speed, and the section is highlighted on the map. It's the quickest way to find out how long that endless scree slope took.

![A section selected on the elevation profile, with its stats](route-tool/selection-en.jpg "The summit push: 502 m of gain at 805 m/h")

### 4. Mark points

This is the part I cared about most. Press **Mark** and click the map or the profile to drop a point. If you click on the route, the point sticks to it; if you click off it (say, a viewpoint across the valley), it stays exactly where you clicked.

Each point has a kind (camp, summit, viewpoint, water, hut, crux, danger, parking or note), a name and an optional note. Points can be dragged, edited and deleted.

![Point editor on the map with the kinds of point](route-tool/marks-en.jpg "Marking a crux, with a note")

The important part: **points are saved in the GPX you download** as standard waypoints, so they show up with their icon on Garmin devices and in most apps. It's a good way to prepare a track for someone doing the route after you: where there's water, where the tricky step is, where it's best to camp. The browser also remembers your points: upload the same files again and they're still there.

You can also mark points from the **Stops** tab: it lists every stop longer than 7 minutes, and one click turns any of them into a point.

### 5. See the terrain in 3D

The **3D** button shows the route on the real terrain, with satellite imagery or a topographic map. You can tilt and rotate the view, and with ▶︎ the camera flies along the route. It's the best way I've found to understand where a route really goes in a tight valley.

![3D view of the route on satellite imagery and terrain](route-tool/3d-en.jpg "The same route in 3D, with the marked points")

### 6. Detailed analysis

Below the map is the full analysis, in tabs:

- **Summary**: effort km, climbing and descent rates, your pace compared with the DIN 33466 standard, time stopped, how much of the route is uphill, flat or downhill, the oxygen available at the high point and the daylight hours that day.
- **Climbs & descents**: finds every significant climb and descent, with its elevation change, average and maximum grade, time and vertical speed. Click one and the map zooms to it.
- **Splits**: time, pace, elevation and grade for every kilometer (or every 2, 5 or 10 km on long routes).
- **Grade** and **Altitude**: how much distance you covered at each grade, and how long you spent in each altitude band.
- **Heart rate**: time in each heart rate zone, if your watch recorded it.

:::carousel{ratio=auto caption="Some of the detailed analysis tabs"}
![Summary tab](route-tool/tab-summary-en.jpg "Summary: effort, pace and conditions")
![Climbs and descents tab](route-tool/tab-climbs-en.jpg "Climbs and descents, found automatically")
![Grade tab](route-tool/tab-grade-en.jpg "Distance by grade range, in the same colors as the map")
:::

Two figures deserve an explanation:

- **Effort km**: the distance plus 1 km per 100 m of ascent and per 150 m of descent. It's the *Leistungskilometer* used by the Swiss Alpine Club, and it makes very different trips comparable: 11 km with 1,700 m of gain "cost" as much as 40 km on the flat.
- **Pace vs. DIN 33466**: the German standard for estimating hiking times (4 km/h on the flat, 300 m/h uphill, 500 m/h downhill). A pace of 0.5× means you take half the standard time. This number is the key to the next tab.

### 7. Plan your next trip

The **Plan** tab estimates the route's timings: you pick a start time, your pace (starting from the one measured on your own track) and how many minutes per hour you rest. It shows the estimated arrival at every marked point, the summit and the finish, and warns you if you'd finish after dark: what time to leave to get back in daylight, and the turnaround time at the summit.

![Planner with start time, pace and estimated timings](route-tool/tab-plan-en.jpg "Estimated times for every point, with sunset taken into account")

It also works with routes you haven't done: upload the GPX of a route you downloaded or drew in a route planner, set the pace to yours, and you get a realistic estimate for your next trip.

## How it's built

For the curious:

- The site is built with [Astro](https://astro.build). The GPX analysis is the same code for my log entries (where it runs when the site is built) and for the tool (where it runs in your browser).
- The map uses [Leaflet](https://leafletjs.com) with maps from [OpenTopoMap](https://opentopomap.org), OpenStreetMap and Esri satellite imagery.
- The 3D view uses [MapLibre GL](https://maplibre.org) with terrain from Mapzen's *Terrain Tiles* hosted on AWS. All free, with no API keys, and only downloaded when you press 3D.
- Sunrise and sunset come from NOAA's solar equations, and the available oxygen from the standard atmosphere.

Later on I want to move it to its own **tools** section, alongside a few others I have in mind. If you use it and think something's missing, let me know!
