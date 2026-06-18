# Technisch Verslag

## 1. Overzicht van de systeemarchitectuur

Het systeem is opgebouwd als een modulaire Duckietown ROS-architectuur waarin verschillende nodes elk een specifieke taak uitvoeren binnen de lokalisatie- en waarnemingsketen. De basis bestaat uit vier hoofdcomponenten: odometrie, visuele SLAM, sensorfusie en semantische perceptie. Daarnaast is er een simulator aanwezig om encoderdata en camerabeelden te genereren, en een visualizer om de resultaten inzichtelijk te maken.

De datastroom begint bij de camera- en encoderinput. De encoderdata wordt gebruikt door de odometrie-node om de positie en orientatie van de Duckiebot te schatten. Tegelijkertijd verwerkt de SLAM-node camerabeelden om visuele kenmerken te detecteren en relatieve camerabeweging te bepalen. De sensor-fusion-node combineert deze informatie met een EKF om een stabielere pose-schatting te krijgen. De semantic-perception-node detecteert AprilTags en duckies en zet deze om naar semantische landmarks en visualisaties.

Deze architectuur is bewust opgesplitst in losse nodes zodat elke taak afzonderlijk te testen, te vervangen en te verbeteren is. De launchstructuur is centraal georganiseerd via een hoofdlaunchfile, waardoor alle onderdelen met een startcommando kunnen worden opgestart. Dat past bij de Duckietown-aanpak en maakt het systeem beter beheersbaar tijdens ontwikkeling en demonstratie.

## 2. Uitleg van de implementatie per taak

### Odometrie

De eerste taak is de odometrie. Hiervoor is een node geimplementeerd die wheel-encoderwaarden ontvangt en deze omzet naar afgelegde afstand. Met een differential-drive kinematisch model worden de positie x, y en orientatie theta geupdatet. De node publiceert de pose als zowel Odometry als PoseStamped, en broadcast daarnaast ook een TF-transform. Hiermee kan de rest van het systeem de robotpositie volgen in het odom-frame.

### Visuele SLAM

De tweede taak is de visuele SLAM. De SLAM-node gebruikt ORB-featuredetectie om herkenbare punten in camerabeelden te vinden en deze tussen opeenvolgende frames te matchen. Op basis van feature-correspondenties wordt een relatieve beweging geschat via de fundamentele en essentiële matrix. Daarnaast wordt een feature map bijgehouden en als PointCloud2 gepubliceerd. De node levert ook een PoseStamped-bericht als camera-motion schatting, zodat deze informatie later gecombineerd kan worden met andere sensoren.

### Sensorfusie

De derde taak is sensorfusie. Hiervoor is een Extended Kalman Filter gebruikt met een toestandsvector van positie en orientatie. De EKF krijgt odometrie- en SLAM-meetwaarden als input en combineert deze tot een gefuseerde pose. De reden voor deze keuze is dat odometrie snel beschikbaar is maar langzaam kan wegdriften, terwijl visuele SLAM gevoeliger is voor ruis en mismatches. Door beide samen te voegen ontstaat een stabielere en robuustere inschatting. De node publiceert het resultaat als fused_pose en fused_odometry.

In deze EKF wordt expliciet met covariantiematrices gewerkt. De toestands-onzekerheid wordt gemodelleerd met de matrix $P$ (state covariance). Tijdens de predictiestap wordt $P$ vergroot met de procesruis $Q$, volgens $P \leftarrow P + Q \cdot \Delta t$. Bij de updates met odometrie en vision worden respectievelijk de meetruis-covariantiematrices $R_{odom}$ en $R_{vision}$ gebruikt. Deze matrices bepalen hoeveel vertrouwen de filter geeft aan elke bron: een lagere waarde betekent meer vertrouwen in de meting.

Concreet houdt de implementatie een $3 \times 3$ covariantiematrix bij voor de toestandsvector $[x, y, \theta]$. De diagonale elementen representeren de varianties op $x$, $y$ en $\theta$, en worden aan het einde ook doorgezet naar `pose.covariance` in het `Odometry`-bericht (op de relevante indexen voor $x$, $y$ en yaw). Hiermee wordt niet alleen een pose geschat, maar ook de onzekerheid van die schatting gerapporteerd.

### Semantische perceptie

De vierde taak is semantische perceptie. Deze node verwerkt camerabeelden voor twee soorten detectie: AprilTags en duckies. AprilTags worden gedetecteerd met OpenCV ArUco/AprilTag-functionaliteit. Duckie-detectie gebeurt via een ONNX-model. De detecties worden vervolgens gekoppeld aan de robotpose zodat landmarks in het odom-frame kunnen worden geplaatst. De node publiceert markers en debugbeelden, zodat de resultaten zowel logisch als visueel te inspecteren zijn.

### Simulator en visualizer

De simulator en visualizer ondersteunen de implementatie. De simulator genereert een gecontroleerde omgeving waarin encoder- en cameradata beschikbaar zijn, zodat de nodes zonder echte hardware getest kunnen worden. De visualizer toont de afzonderlijke odometrie, gefuseerde pose en SLAM-features in een grafiek, waardoor de werking van het systeem snel te beoordelen is.

## 3. Ontwerpkeuzes en afwegingen

Een belangrijke ontwerpkeuze was het opdelen van het systeem in losse nodes in plaats van een grote monolithische applicatie. Dit maakt de code overzichtelijker, vergroot de testbaarheid en sluit beter aan op het ROS/Duckietown-ecosysteem. Ook is gekozen voor duidelijke topic-scheiding, zodat elke module alleen de informatie ontvangt die nodig is voor zijn eigen taak.

Voor de pose-estimatie is gekozen voor een EKF in plaats van alleen odometrie of alleen visuele SLAM. Die keuze is gemaakt omdat odometrie goedkoop en stabiel is op korte termijn, terwijl visuele informatie nuttig is om drift te corrigeren. De EKF is een praktische tussenoplossing die goed past bij een real-time robottoepassing.

Bij de semantische perceptie is gekozen voor een combinatie van klassieke computer vision en een ONNX-model. AprilTags zijn betrouwbaar voor vaste landmarks, terwijl objectdetectie nuttig is voor dynamische of semantische objecten zoals duckies. De combinatie geeft meer informatie dan alleen geometrische lokalisatie.

Daarnaast is de implementatie aangepast naar het Duckietown/DTROS-patroon. Dat is belangrijk omdat Duckietown nodes vaak verwachten dat modules binnen de DTROS-structuur draaien en goed integreren met de rest van het systeem. Daarmee wordt de software beter bruikbaar op echte Duckiebots en beter compatibel met de gangbare ontwikkelomgeving.

## 4. Beperkingen en faalscenario's

Hoewel het systeem functioneel is, zijn er duidelijke beperkingen. De SLAM-node gebruikt monoscopische visuele features en kan daardoor beperkt zijn in absolute schaal en gevoelig voor beeldkwaliteit. Slechte belichting, motion blur of weinig herkenbare kenmerken kunnen leiden tot foutieve matches of instabiele pose-schattingen.

De EKF-fusie is afhankelijk van de kwaliteit van de invoer. Als zowel odometrie als vision tijdelijk onbetrouwbaar zijn, dan kan de gefuseerde pose alsnog afdrijven. De filterparameters zijn relatief simpel ingesteld en zijn nog niet uitgebreid gekalibreerd voor alle mogelijke situaties.

De semantische perceptie is eveneens afhankelijk van beeldkwaliteit en modelbetrouwbaarheid. AprilTag-detectie faalt bij occlusie, scheve kijkhoeken of slechte verlichting. Het ONNX-model voor duckies kan fout-positieven of gemiste detecties geven, vooral wanneer de invoerbeelden afwijken van de trainingsdata.

Een ander aandachtspunt is de robuustheid op echte hardware. De huidige codebase is functioneel opgezet, maar niet elk onderdeel is even sterk geoptimaliseerd voor een beperkte embedded omgeving. Denk hierbij aan verwerkingsbelasting, topic-synchronisatie en de beschikbaarheid van alle benodigde Duckietown-pakketten. Ook kan een mismatch tussen simulatorparameters en echte robotparameters invloed hebben op de nauwkeurigheid.

Tot slot is het systeem gevoelig voor topic- en frame-afstemming. Als de frames niet consistent zijn of een launchconfiguratie ontbreekt, kunnen nodes wel draaien maar geen bruikbare data met elkaar uitwisselen. Dat is vooral relevant bij integratie op Duckiebots, waar de conventies van Duckietown strikt gevolgd moeten worden.

## 5. Conclusie

De software vormt een modulaire lokalisatie- en perceptieoplossing voor Duckietown, met odometrie, visuele SLAM, sensorfusie en semantische detectie als kernonderdelen. De gekozen architectuur maakt het systeem uitbreidbaar en overzichtelijk. Tegelijkertijd zijn er nog beperkingen in robuustheid, parameterafstemming en hardware-afhankelijk gedrag. Voor een volgende stap zou verdere DTROS-integratie, parameterkalibratie en robuustheidstesten op echte hardware de belangrijkste verbeterpunten zijn.
