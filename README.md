# Qube
-----
# Innledning (om prosjektet):
I emnet Mekatronikk og robotikk har vi (Jon Håvard Sylte og Trym Vannebo) nylig gjennomført et mini prosjekt der vi skulle ved hjelp av ROS2 styre en Quanzer Qube. 
Prosjektet gikk ut på å styre vinkelen til Quben ved å bruke en terminal og/eller en GUI. I tillegg til dette skulle vi lage en PID-kontroller som skulle regulere og gi ut et hastighets-pådrag. I tillegg til den fysiske Qube-roboten skulle vi også lage en simulator av roboten, som skulle ha en visualisering i RViz. For å gjennomføre dette måtte vi totalt ha fire (fem) pakker med forskjellige funksjoner og systemer som sammarbeidet med hverandre. Pakkene vi ende opp med er Qube_description, Qube_driver, Qube_bringup, Qube_controller og Pid_controller_msgs. Qube_driver pakken var laget på forhånd og alt som trengtes for denne pakken var å laste den ned og å ha ROS2 Control installert for at pakken skulle fungere.

# Qube_description:
Denne pakken inneholder 2 URDF-filer, som har hver sin funksjon. De to URDF-filene er: 

    qube.macro.xacro
    qube.urdf.xacro
-Macro-filen

Macro-filen (qube.macro.xacro) inneholder beskrivelsen av quben. I denne filen blir en digital versjon av quben laget, ved å først bygge de ulike delene til roboten, før de derreter blir satt sammen med ulike joints. Et eksempel på dette kan bli sett i kode utklippet under, der vi først beskriver den røde roterende disken (Rotor link) til kuben og den hvite viseren (Angle link) som skal indikere vinklen til quben, etterfulgt av hvordan disse to skal sitte sammen med en joint.

        <!-- Rotor link -->
        <link name="${prefix}rotor_link">
            <visual>
                <geometry>
                    <cylinder radius="${cylinder_radius}" length="${cylinder_height}"/>
                </geometry>
                <material name="red">
                    <color rgba="1 0 0 1"/>
                </material>
            </visual>
        </link>
        
        <!-- Angle link -->
        <link name="${prefix}angle_link">
            <visual>
                <geometry>
                    <box size="${angle_box_size}"/>
                </geometry>
                <material name="white">
                    <color rgba="1 1 1 1"/>
                </material>
            </visual>
        </link>

        <joint name="${prefix}motor_joint" type="revolute">
            <parent link="${prefix}stator_link"/>
            <child link="${prefix}rotor_link"/>
            <origin xyz="0 0 ${box_size/2}" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
            <limit effort="1.0" lower="-3.14" upper="3.14" velocity="1.0"/>
        </joint>

-Scene-filen

Scene-filen (qube.urdf.xacro) fungerer som en scene for roboten, ved å lage en link mellom quben vi beskrev i macro-filen og en scene. Scenen i denne filen er en veldig simpel scene der quben står i origo. Ved å sette opp pakken på denne måten, så har vi separert funksjonene til de to filene, der den ene filen beskriver quben, mens den andre legger den inn i en scene, slik at beskrivelsen kan bli visualisert. Ved å sette det opp på denne måten så blir også filen der vi beskriver quben gjenbrukbar, slik at den kan bli brukt videre i andre URDF-filer.

I tillegg til de to URDF-filene, så inneholder denne pakken også en launch-fil. Denne launch-filen blir brukt senere i qube_bringup pakken, der den blir brukt til å starte opp macro- og scene-filen slik at vi får en digital versjon av quben når vi starter opp hovedprogrammet. Lanch filen for macro- og scene-filen er:

        view_qube.launch.py

# Pid_controller_msgs
Er denne pakken ferdig? Og hva gjør den/skal vi skrive noe om den?

# Qube_bringup:
Denne pakken inneholder en launch- og en konfigurasjon-fil som fletter sammen Qube-systemet. Konfigurasjonsfilen er en URDF-fil som heter controlled_qube.urdf.xacro og den er relativt lik scene-filen fra qube_description pakken, bortsett fra at konfigurasjonsfilen inneholder mer en det scene-filen gjør. I tillegg til innholdet til scene-filen, så inkluderer konfigurasjonsfilen også qube_driver.ros2_control.xacro filen, som (skriv mer her når kode er lastet opp). 
I tillegg til dette så inneholder konfigurasjonsfilen tre macro argumenter: baud_rate, device og simulation. 

Disse argumentene er det som blir brukt for å bestemme om vi skal simulere eller koble til den fysiske quben, samt hvordan dette skal gjøres. Simulation argumentet er det som bestemmer om vi skal simulere quben digitalt ved å sette argumentet til "True" eller om programmet skal koble seg til den fyske kuben ved å sette simulation til "False". Device argumentet er det som bestemmer hvilen USB-port som programmet skal bruke for å styre quben. Baud_rate er argumentet som bestemmer hvor ofte/fort (er dette riktig?) programmet skal sende signaler til quben. Standaren for baud_rate argumentet som ble brukt i dette prosjketet var 115200.

Som sagt, så hadde vi også en launch-fil for denne pakken. Denne lauch-filen ble brukt til å starte opp qube_driver.launch.py fra qube_driver-pakken, rviz og robot state controller. Dette gjør at alle de essensielle delene av det samlede programmet starter opp med kun en kommando, som gjør at det ferdige programmet blir lettere å bruke.

# Qube_controller:
Denne pakken inneholder PID-kontrolleren for å regulere Quben. (Hva skal vi skrive her?, Kommer ikke på hva vi kan skrive om denne pakken) 
