# RoboRacer — The Story So Far (explained simply)

A plain-language, merged write-up of both work sessions on **2026-06-30**, meant
for a new team member. It explains *what* we did and *why*, not just the commands.
For the exact commands, use `guide.md` (run steps) and `session.md` / `session2.md`
(detailed logs).

---

## What is this thing?

It's a **1/10-scale self-driving race car** (an "F1TENTH" / RoboRacer). On board:

- A small computer (**NVIDIA Jetson**) running Linux + **ROS 2** (ROS = the
  software framework robots use; think of it as the messaging system that lets
  the sensors, brain, and motors talk to each other).
- A **LiDAR** — a spinning laser that measures distance in every direction. It's
  how the car "sees" walls.
- A **VESC** — the motor controller that spins the wheels and reports how far
  they've turned (this estimate of movement is called **odometry**).
- A **gamepad** for manual driving, and a Wi-Fi link so we can SSH in from a
  laptop.

Two student groups share this one car. Our group brought our **simulation** code
onto the real car for the first time.

## The goal

Make the car drive **by itself**: you put a **start** and an **end** point on a
map, and it figures out a route and drives there. To do that a robot needs four
things, and it helps to know the words:

1. **A map** — a picture of the track's walls. We build it with **SLAM**
   (Simultaneous Localization And Mapping): you drive the car around once while
   the software stitches the laser scans into a map.
2. **Localization** — knowing *where the car is on that map* right now. (Wheel
   odometry alone slowly drifts, so we match the live laser scan against the map.)
3. **A planner** — the part that takes "I'm here, I want to go there" and draws a
   **path** (a line of waypoints) that avoids the walls.
4. **A controller** — the part that *follows* that path by steering and setting
   speed. Planner = "draw the route"; controller = "drive the route." They are
   **two different jobs**, and this distinction turned out to matter a lot.

We use a popular ROS toolkit called **Nav2** that provides the planner,
controller, localization, and the glue around them.

A safety note built into the car: a **mux** (a priority switch) decides whose
commands reach the motor. The **gamepad always wins** over the autonomy, so you
can grab the controller and override the car instantly.

---

## Session 1 (morning): get it moving, and make a map

- Connected to the car over Wi-Fi (SSH) and **backed up the other group's work**
  so we couldn't accidentally destroy it.
- Copied our code onto the car and **compiled** it.
- Checked the sensors: the laser and wheel odometry were publishing correctly,
  and the car's internal "where are my parts" coordinate system (**TF**, the
  transform tree) was healthy.
- Fixed an annoying quirk: the car has no clock battery, so every boot it thinks
  it's **1970**. A wrong clock confuses time-stamped sensor data, so we set it.
- **Drove the car for the first time** with the gamepad — the full chain worked:
  gamepad → mux → motor controller → wheels.
- Did a **SLAM run**: drove a slow lap and saved a map of the track
  (called `my_track`).

Where it stalled: to drive autonomously we needed **Nav2**, but installing it
needs internet, and the car's only internet was a phone hotspot that was too slow
that day. So the Nav2 install was left unfinished.

## Session 2 (afternoon): give it a brain, and prepare autonomy

1. **Finished installing Nav2.** We got the car online through a 5 GHz phone
   hotspot and installed the full Nav2 toolkit (38 software packages). Two tricks
   were needed: temporarily set the clock *ahead* of real time (otherwise the
   software repository refused to install, thinking the files were "from the
   future"), and run the download **detached** so the flaky connection couldn't
   interrupt it.

2. **Discovered an important gap.** We searched the whole codebase and found that
   our project has a **controller** (the path-follower) and a small relay, but
   **no planner of its own** — the route had always been drawn by Nav2. So
   removing Nav2 was never an option; we needed it.

3. **Chose how to wire it up.** Our team's own controller has a catch: it assumes
   the car's position is already given in the *map's* coordinate system, and it
   does no conversion. On the real car the position comes in a *drifting* local
   coordinate system, so the two wouldn't line up and the car would track the
   path wrong. Nav2's own controller handles this conversion correctly. So for
   the **first** real run we let **Nav2 drive end-to-end** (its planner *and* its
   controller), and just convert its output into the steering/throttle format the
   car expects. Using our team's controller is saved for a later "Phase 2" once a
   small position-converter is added.

4. **Measured the real car's numbers** and baked them into the config: wheelbase
   25 cm, max steering ~0.32 rad (so it can't turn tighter than ~0.75 m radius),
   and a hardware top speed of about 2 m/s. For safety we **capped the first run
   at 0.5 m/s** — a slow walk.

5. **Prepared everything as ready-to-run files** (config, launch file, helper
   scripts) and wrote a **step-by-step runbook** (`guide.md`). We did **not** run
   the car this session — everything is staged for you to execute.

6. **Built a debugging/logging system.** The autonomous pipeline has seven links
   in a chain (sensors → coordinate frames → map → localization → planner →
   commands → motors). A single script, `rr_healthcheck.sh`, tests each link and
   prints PASS/FAIL with a hint, so when something breaks you know *exactly which
   link* failed instead of guessing. All logs are saved to one folder for later.

7. **Saved everything to GitHub** on a branch called `Hardware`, and **tidied the
   car**, removing our temporary files while keeping Nav2 and the hardware setup.

---

## Where things stand now

- The car **drives manually** and we have a **map** of the track.
- **Nav2 is fully installed**, and a complete, safety-capped autonomous
  configuration is **prepared and saved on GitHub** (`Hardware` branch).
- It is **not yet deployed/built/tested on the car** — that's the next step.
- The first autonomous attempt is pre-set to a safe **0.5 m/s**.

## What's next (in order)

1. Copy the prepared files to the car and compile them.
2. Follow `guide.md`: start the sensors, run the health check, (re-)build the map
   if needed, launch Nav2, tell the car its **start** position and an **end**
   goal, and watch it drive itself slowly.
3. Use the health-check script at each step; only raise the speed after a clean
   slow lap.
4. Later (Phase 2): add the small position-converter so the car can use *our
   team's* controller instead of Nav2's.

## Things that bite (keep these in mind)

- The car's **clock resets to 1970** every boot — set it, especially before
  installing software or recording data.
- The car only sees **5 GHz** Wi-Fi, and the phone-hotspot internet is slow.
- The **gamepad overrides** the autonomy — keep it in hand as your stop button.
- Don't store the code's git folder inside **OneDrive** — during session 2 the
  synced folder vanished mid-task. Keep it in a plain local folder.
