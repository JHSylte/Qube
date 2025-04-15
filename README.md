# Qube
-----
# Innledning (om prosjektet):
I emnet Mekatronikk og robotikk har vi (Jon Håvard Sylte og Trym Vannebo) nylig gjennomført et mini prosjekt der vi skulle ved hjelp av ROS2 styre en Quanzer Qube. 
Prosjektet gikk ut på å styre vinkelen til Quben ved å bruke en terminal og/eller en GUI. I tillegg til dette skulle vi lage en PID-kontroller som skulle regulere og gi ut et hastighets-pådrag. I tillegg til den fysiske Qube-roboten skulle vi også lage en simulator av roboten, som skulle ha en visualisering i RViz. For å gjennomføre dette måtte vi totalt ha fire pakker med forskjellige funksjoner og systemer som sammarbeidet med hverandre. Pakkene vi ende opp med er Qube_description, Qube_driver, Qube_bringup og Qube_controller.

# Qube_description:
Beskrivelse av quben...
Denne pakken inneholder 2 filer:

    qube.macro.xacro
    qube.urdf.xacro
Macro

Urdf
# Qube_driver:
kommunikasjonsgrensesnittet med den fysiske Quben...
# Qube_bringup:
launch- og konfigurasjonsfiler
# Qube_controller:
PID-kontroller som regulerer roboten/Quben...
