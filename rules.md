# COVAPSY 2026 Rules — Researched Summary

**Official rules distilled into an implementation-oriented bullet document**  
**Event target:** 6th edition CoVAPSy, ENS Paris-Saclay  
**Prepared:** 2026-03-08  
**Audience:** autonomous 1/10-scale racing team / implementation planning

## Scope and source status

- I did not find a separate 2026 PDF rulebook. The current official source of truth is the COVAPSY rules page on the official event website, supported by the official homepage and the official GitHub repository.
- This document separates what is explicitly stated in the official rules from what is only a practical implication for your build.
- Where the published rules are slightly inconsistent, this document flags the inconsistency instead of silently smoothing it over.

## 1. High-impact takeaways

- The 2026 edition is scheduled for Saturday, April 11, 2026, at ENS Paris-Saclay. [S2]
- Only start and stop signals may be sent from the team to the vehicle during the race; telemetry from the vehicle back to the team is allowed. Sending behavioral commands to the vehicle is disqualifying. [S1]
- A 2026-specific relaxation exists: an off-car trackside computer is allowed for remote computation, but the team may not touch that computer once the start is announced. [S1]
- The car must be built on a Tamiya TT-02 chassis/motor kit, use a propulsion battery limited to 7.2 V NiMH and 5000 mAh maximum, and be able to drive both forward and reverse. [S1]
- The track layout is unknown until race day. Preloading the shape of the track into the car is forbidden; however, the car may learn the track through up to 3 setup laps. [S1]
- Homologation is not just a dimensional/safety check: the car must also demonstrate remote start/stop, straight-plus-turn driving without touching borders, reversing when blocked, and obstacle avoidance. [S1]
- The rear of the car must be LiDAR-visible. If the rear paint is not detectable, the referees may add off-white tape to the rear of the car. [S1]

## 2. Official rules in bullet points

### 2.1 Vehicle rules

- The vehicle must use a Tamiya TT-02 chassis/motor kit. [S1]
- The propulsion battery must be NiMH, 7.2 V, with a maximum capacity of 5000 mAh. [S1]
- A separate secondary battery is allowed for electronics, regardless of chemistry. [S1]
- Feeding the motor with a voltage higher than the propulsion battery voltage is not allowed. [S1]
- The body shell must cover more than 80% of the car; the jury estimates this informally rather than via a strict measurement process. [S1]
- Complete car dimensions, including all sensors and actuators, must stay within these limits: length strictly between 400 mm and 480 mm, width strictly between 160 mm and 190 mm, and height strictly between 115 mm and 190 mm. [S1, S4]
- The rear of the vehicle must present at least one solid rectangle 150 mm wide by 110 mm high; only a small gap under 10 mm in height is tolerated for ground clearance. [S1, S4]
- Transparent body shells are forbidden. A body whose main color is green, red, or gray is also forbidden. [S1]
- Rear visibility is checked with an RP-Lidar A2M12 or A2M8. If the rear is not detected, the organizers may add off-white tape to make it detectable. [S1]
- Removing the front-wheel drive is allowed if you want to trade traction for a tighter steering radius. [S1]
- The vehicle must be able to operate in forward and reverse. [S1]
- Minor steering-mechanics upgrades using commercial aluminum parts are allowed, but major chassis modifications beyond a few drilled holes require a prior request and acceptance by the other participating institutions; the request must include references or plans. [S1]
- A "typical car" kit is available as a resource, but it is a support option rather than a mandatory architecture. [S1, S4]

### 2.2 Team-to-car communication and off-board computing

- During competition, team-to-car communication must be limited to a start signal and a stop signal. [S1]
- Vehicle-to-team telemetry is allowed. [S1]
- Any command that modifies vehicle behavior during the run causes disqualification. [S1]
- For the 2026 edition, off-board computation on a trackside computer is explicitly allowed. [S1]
- Once the start is announced, the team may not touch the trackside computer. [S1]
- Practical implication: remote visualization or map optimization is compatible with the published rules only if the car remains autonomous after the start and no human-in-the-loop driving/planning commands are sent. [Inference from S1]

### 2.3 Track rules

- The shape of the track is unknown before race day. [S1]
- Providing information about the track shape to the vehicle is forbidden. Referees may verify compliance, for example on an auxiliary track. [S1]
- The car may learn the track by performing up to 3 setup laps. [S1]
- The track uses 200 mm-high borders. [S1]
- In one travel direction, the right border is green and the left border is red; in the opposite direction the color roles reverse. [S1]
- Borders are made from straight segments and circular arcs with curvature radius R >= 400 mm. [S1]
- The floor is a gray linoleum/PVC surface. [S1]
- The track width is greater than 800 mm at every point, but the track may contain obstacles inside the drivable space. [S1]
- The published color references are approximate: green RAL 6037, red RAL 3020, and a gray Gerflor flooring reference for the ground. The rules explicitly warn that borders may be marked by impacts, so perception must tolerate wear and variation. [S1]

### 2.4 Homologation requirements

- The published rules say homologation occurs in "4 steps", but the page actually lists 5 checks. For planning, treat all 5 listed checks as required. [S1]
- Check 1: validation of vehicle elements such as dimensions, battery, chassis, and LiDAR-detectable rear color/finish. [S1]
- Check 2: validation of remote start and remote stop. [S1]
- Check 3: demonstration that the vehicle can drive along a straight section plus a turn without touching the borders. [S1]
- Check 4: demonstration that the vehicle can reverse and restart when blocked against an obstacle, provided there is no vehicle behind it. [S1]
- Check 5: demonstration that the vehicle can avoid an obstacle approximately the size of a car on a section of track. [S1]
- The referees may still grant homologation with penalties for light rule violations; the page gives remote stop deficiencies and lack of reverse operation as examples. [S1]

### 2.5 Setup laps and SLAM-specific allowance

- For both races and qualifications, a car that uses SLAM may perform up to 3 setup laps without obstacles. [S1]
- Only after the car is placed on the starting grid are the obstacles or opponent vehicles added. [S1]
- Practical implication: mapping/localization during setup laps is explicitly contemplated by the official rules. [Inference from S1]

### 2.6 Qualification phase 1 — time trial

- Qualification phase 1 consists of 2 solo runs on track A with fixed obstacles roughly the size of a car. [S1]
- Each run is 2 laps. [S1]
- One run is clockwise and the other is counterclockwise. [S1]
- The best of the 2 times is retained. [S1]
- If the car fails to finish 2 laps in both runs, a time of 120 s is assigned if the car completed 1 lap, and 180 s if it did not even finish 1 lap. [S1]
- After this phase, the first car gets 25 points, the second 24, and so on down to a minimum of 5 points for any car with a time under 120 s. A car that completed only 1 lap and is assigned 120 s gets 1 point. [S1]

### 2.7 Qualification phase 2 — heat races

- Qualification phase 2 consists of 2 heat races with groups of 4 to 8 cars. [S1]
- The cars run 3 laps on track B and 3 laps on track C. [S1]
- One of those races is clockwise and the other is counterclockwise. [S1]
- The N pools are built from qualification ranking using the published modulo-style rule described on the official page. [S1]
- Each race awards 10 points to 1st place, 8 to 2nd, 6 to 3rd, 4 to cars that completed all 3 laps, and 1 to cars that did not finish. [S1]

### 2.8 Common procedure during race heats and finals

- Teams have 3 minutes to install their vehicle on the track. [S1]
- Vehicles are placed on the starting grid according to qualification results. [S1]
- After all teams declare they are ready, touching the vehicles is forbidden. [S1]
- The start signal is given orally by the referee. [S1]
- Arrival order is recorded after a predefined number of laps: by default 3 laps in qualification races and 5 laps in final races. [S1]
- A car that does not complete the defined number of laps is not classified for that race. [S1]

### 2.9 Removals, penalties, and disqualification conditions

- A vehicle showing clearly aggressive behavior toward opponents is removed/disqualified from the race. [S1]
- A vehicle that intentionally prevents another car from overtaking is also removed/disqualified. [S1]
- A vehicle immobilized on track for more than 10 seconds, when it is not being blocked by another car, is removed from the track. [S1]
- A vehicle that travels more than 2 meters in the wrong direction is removed from the track. [S1]
- Sending team-to-car commands beyond start/stop is disqualifying at the rules level. [S1]
- The rules also allow homologation penalties for light infractions before racing starts. [S1]

### 2.10 Final phases

- After qualifications, each car has 2 scores: coursesQualif (sum of the 2 qualification-race scores) and CLM+courseQualif (that same total plus the time-trial points). [S1]
- The finals are organized as 2 rounds ("manches"), each made of 3 races. [S1]
- The top 2 cars from each pool go to race 1 of each final round; the 3rd and 4th from each pool go to race 2; the others go to race 3. [S1]
- Starting grid order is determined by qualification ranking, using coursesQualif first and CLM+courseQualif as the tie-breaker for ex aequo cases. [S1]
- Final-race points are awarded as follows: 25, 18, 15, 12, 10, 8, 6, 4, 2, 1 from 1st through 10th place; a car that does not finish the required laps gets 0 points. [S1]
- After ranking the cars from the first race, points continue to be assigned to the cars from the second race as described on the official page. [S1]
- The final overall classification uses only the points from the 2 final races. [S1]
- If there is a tie in the final classification, CLM+courseQualif is used first; if still tied, coursesQualif is used. [S1]

### 2.11 Prizes

- The official page states that prizes are awarded to 1st, 2nd, and 3rd place overall. [S1]
- There is also an innovation prize. [S1]
- There is also a prize for the best-ranked team among the licence-level entrants. [S1]

## 3. What the rules do not mandate

- The rules do not mandate Raspberry Pi 5, STM32, ROS 2, Linux, Webots, RealSense, RPLIDAR as your onboard stack, or any particular software architecture. Those are engineering choices, not regulatory requirements. [S1, S4, S5]
- The rules do not mandate SLAM specifically; they only allow up to 3 setup laps and explicitly mention SLAM-capable cars in that context. [S1]
- The rules do not mandate a camera. They also do not mandate a LiDAR as the primary navigation sensor, although rear detectability is checked with RP-Lidar hardware. [S1]
- The typical car page and simulator page are official resources for getting started, but they are not the competition rules themselves. [S4, S5]

## 4. Build-critical implications for your implementation plan

- Your car must have a reliable remote start/stop path that works under homologation conditions. This is not optional. [S1]
- Reverse maneuver capability is a hard competition requirement, not a "nice to have". [S1]
- Obstacle avoidance must work both in homologation and during racing. A pure lane-following solution without obstacle handling is non-compliant. [S1]
- Unknown-track operation matters: you cannot rely on a preloaded official map. Any map-based racing mode must be learned on site within the allowed setup-lap framework. [S1]
- Human-in-the-loop trackside control is not allowed. If you use offboard compute in 2026, the car still needs to remain autonomous and safe after the start signal. [S1]
- Rear detectability is a real rule item, so body design and paint choice are part of compliance. [S1]
- Because aggressive blocking and more than 2 m of wrong-way travel cause removal, recovery logic must be conservative and direction-aware. [S1]
- Because a car sitting still for more than 10 s without being blocked is removed, deadlock detection and fallback behavior matter strategically, not just technically. [S1]

## 5. Open points / ambiguities to verify early

- The official page describes homologation as "4 steps" but lists 5 checks. Treat the list itself as authoritative unless the organizers clarify otherwise. [S1]
- The page gives dimensional bounds through an image rather than a text table. Keep your mechanical design clearly inside the limits rather than exactly at a boundary. [S1, S4]
- The wording about final-round point continuation across races is concise and could be read in more than one procedural way. It does not change the safe engineering conclusion that finishing and classification are critical. [S1]
- Because the official site notes that the rules evolve year to year, re-check the rules page shortly before final freeze and before travel. [S1]

## 6. Sources reviewed

- **[S1]** Official rules page — COVAPSY / ENS Paris-Saclay — https://ajuton-ens.github.io/CourseVoituresAutonomesSaclay/Reglement_CourseVoituresAutonomes/
- **[S2]** Official event homepage — confirms 2026 date — https://ajuton-ens.github.io/CourseVoituresAutonomesSaclay/
- **[S3]** Official GitHub repository — rules/resources host — https://github.com/ajuton-ens/CourseVoituresAutonomesSaclay
- **[S4]** Official "typical car" page — supportive resource, plus mechanical context — https://ajuton-ens.github.io/CourseVoituresAutonomesSaclay/voiture_type/
- **[S5]** Official simulator page — supportive resource for 2025–2026 development — https://ajuton-ens.github.io/CourseVoituresAutonomesSaclay/simulateur/