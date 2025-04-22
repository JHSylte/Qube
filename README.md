# Qube
-----
# Innledning:
I emnet Mekatronikk og robotikk har vi (Jon Håvard Sylte og Trym Vannebo) nylig gjennomført et mini prosjekt der vi skulle ved hjelp av ROS2 styre en Quanzer Qube. 
Prosjektet gikk ut på å styre vinkelen til Quben ved å bruke en terminal og/eller en GUI. I tillegg til dette skulle vi lage en PID-kontroller som skulle regulere og gi ut et hastighets-pådrag. I tillegg til den fysiske Qube-roboten skulle vi også lage en simulator av roboten, som skulle ha en visualisering i RViz. For å gjennomføre dette måtte vi totalt ha fire pakker med forskjellige funksjoner og systemer som sammarbeidet med hverandre. Pakkene vi ende opp med er Qube_description, Qube_driver, Qube_bringup og Qube_controller. 

# Qube_description:
Denne pakken inneholder 2 URDF-filer, som har hver sin funksjon. De to URDF-filene er: 

    qube.macro.xacro
    qube.urdf.xacro
-Macro-filen (qube.macro.xacro)

Macro-filen inneholder beskrivelsen av quben. I denne filen blir en digital versjon av quben laget, ved å først bygge de ulike delene til roboten, før de derreter blir satt sammen med ulike joints.

-Scene-filen (qube.urdf.xacro)

Scene-filen fungerer som en scene for roboten, ved å lage en link mellom quben vi beskrev i macro-filen og en scene. Scenen i denne filen er en veldig simpel scene der quben står i origo. Ved å sette opp pakken på denne måten, så får man separert funksjonene til de to filene, der den ene filen beskriver quben, mens den andre legger den inn i en scene, noe som gjør at beskrivelsen kan bli visualisert. Ved å sette det opp på denne måten så blir også filen med beskrivelsen av quben gjenbrukbar, slik at den kan bli brukt videre i andre URDF-filer.

-Launch-fil

I tillegg til de to URDF-filene, så inneholder denne pakken også en launch-fil. Denne launch-filen blir brukt senere i qube_bringup pakken, der den blir brukt til å starte opp macro- og scene-filen slik at man får en digital versjon av quben når man starter opp hovedprogrammet. Lanch filen for macro- og scene-filen er:

    view_qube.launch.py

# Qube_Driver:
Denne pakken har vi ikke laget selv. Som en del av dette prosjektet, så var denne pakken allerede laget og det enste som måtte gjøres var å laste den ned fra: https://github.com/adamleon/qube_driver
Videre for at denne pakken skulle fungere så må ROS2 Control installert være installert, som kan bli gjort ved å skrive inn denne koden i en terminal:
    
    sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers


# Qube_bringup:
Denne pakken inneholder en launch- og en konfigurasjon-fil som fletter sammen Qube-systemet. Konfigurasjonsfilen er en URDF-fil som heter controlled_qube.urdf.xacro og den er relativt lik scene-filen fra qube_description pakken, bortsett fra at konfigurasjonsfilen inneholder mer en det scene-filen gjør. I tillegg til innholdet til scene-filen, så inkluderer konfigurasjonsfilen også qube_driver.ros2_control.xacro filen som setter opp ROS 2 kontrollgrensesnittet. Videre så inneholder konfigurasjonsfilen tre macro argumenter: baud_rate, device og simulation. 

Disse argumentene er det som blir brukt for å bestemme om man skal simulere eller koble til den fysiske quben, samt hvordan dette skal gjøres. Simulation argumentet er det som bestemmer om man skal simulere quben digitalt ved å sette argumentet til "True" eller om programmet skal koble seg til den fyske kuben ved å sette simulation til "False". Device argumentet er det som bestemmer hvilen USB-port som programmet skal bruke for å styre quben. Baud_rate er argumentet som bestemmer hvor ofte/fort programmet skal sende signaler til quben. Standaren for baud_rate argumentet som ble brukt i dette prosjketet var 115200.

Som sagt, så er det også en launch-fil for denne pakken. Denne lauch-filen blir brukt til å starte opp qube_driver.launch.py fra qube_driver-pakken, rviz og robot state controller. Dette gjør at alle de essensielle delene av det samlede programmet starter opp med kun en kommando, som gjør at det ferdige programmet blir lettere å bruke.

# Qube_controller:
Denne pakken implementerer en PID-kontroller for å styre Quben ved hjelp av ROS2. Målet er å justere robotens bevegelse slik at den holder en ønsket posisjon (referanse). En viktig funksjon i pakken er muligheten til å justere PID-parametrene (P, I, D) samt referanseverdien i sanntid. Dette gjøres ved hjelp av ROS2-parametere. Parametrene kan settes via ros2 param set-kommandoen eller programmatisk, og de endringene du gjør vil påvirke PID-kontrollerens respons umiddelbart. Dette gir brukeren fleksibilitet til å justere kontrollsystemet etter behov, for eksempel for å håndtere forskjellige typer dynamikk i roboten eller for å finjustere ytelsen.

# Bruk av programmet:
Nå som vi har gått gjennom hva de ulike pakkene i programmet gjør, så kan vi gå inn på hvordan man kan bruke det ferdige programmet der alle pakkene er blitt satt sammen. 

-Launch

For å starte opp programmet med default-parameterene så kan man skrive inn denne kommandoen i terminalen (etter at pakken er bygget):
   
    ros2 launch qube_bringup bringup.launch.py
Dette vil starte opp programmet med launch parameterene:

    simulation = true
    baud_rate = 115200
    device = /dev/ttyACM0    

Dersom man vil starte opp programmet med andre launch parametere, som for eksempel å koblectil den fysiske Quben, så kan man lett legge til argumentene for dette på slutten av kommandoen slik som dette:

    ros2 launch qube_bringup bringup.launch.py use_sim:=false baudrate:=115200 device:=/dev/ttyACM1

-PID-parametere

Default paramteren for PID-regulatoren og referanseverdien er:

    P = 10.0
    I = 3.0
    D = 0.3
    Setpoint = 0.0
Disse parameterne kan også endres på etter at programmet har startet. For å endre på parameterene må man åpen en annen terminal, hvor man kan bruke ros2 param set-kommandoen til å beste parameteren slik som dette, der X er en integer:

    ros2 param set /qube_controller p X
    ros2 param set /qube_controller i X
    ros2 param set /qube_controller d X
    ros2 param set /qube_controller reference X


