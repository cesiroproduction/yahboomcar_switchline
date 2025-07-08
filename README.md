Documentație Tehnică: Proiect arm_autopilot
## 1. Prezentare Generală a Proiectului
Proiectul arm_autopilot este un pachet software dezvoltat în cadrul ROS (Robot Operating System), conceput pentru a oferi capabilități de navigație autonomă unui robot mobil (Yahboom). Funcționalitatea centrală constă în detectarea și urmărirea unor linii colorate de pe o suprafață, folosind o cameră video și algoritmi de Computer Vision (OpenCV).

Sistemul este robust, permițând nu doar urmărirea unei singure linii, ci și comutarea inteligentă între două trasee de culori diferite la comanda utilizatorului. Controlul mișcării robotului este realizat printr-un controler PID (Proporțional-Integral-Derivativ), asigurând o deplasare fluidă și precisă. Proiectul include și un mod de calibrare interactivă a culorilor și a parametrilor de mișcare.

Funcționalități Cheie:

Urmărirea automată a liniilor pe baza culorii.

Control precis al vitezei unghiulare printr-un regulator PID.

Comutare dinamică între două culori de traseu (red și yellow) la apăsarea tastei 'S'.

Rutină inteligentă de căutare atunci când linia este pierdută (robotul se întoarce spre ultima poziție cunoscută).

Sistem de calibrare a culorilor (valori HSV) cu salvare persistentă în fișier YAML.

Configurare dinamică a parametrilor (PID, viteză) în timp real folosind rqt_reconfigure.

## 2. Structura Fișierelor și Rolul Acestora
Proiectul este organizat conform standardelor unui pachet ROS.

package.xml și CMakeLists.txt

Rol: Fișiere standard de configurare ROS. package.xml definește metadatele pachetului, cum ar fi numele, versiunea și, cel mai important, dependențele software (rospy, opencv, dynamic_reconfigure). CMakeLists.txt conține regulile pentru compilarea și instalarea pachetului în mediul ROS.

launch/arm_autopilot.launch

Rol: Fișier de lansare ROS. Acesta este modul recomandat de a porni nodul principal al proiectului. Definește ce nod (autopilot_main.py) trebuie rulat și poate seta parametri inițiali.

cfg/autopilot.cfg

Rol: Fișier de configurare pentru Dynamic Reconfigure. Aici sunt definiți toți parametrii care pot fi modificați în timp real în timp ce robotul funcționează. Aceștia includ:

Câștigurile PID: Kp, Ki, Kd.

Viteza liniară: linear.

Valorile HSV: Hmin, Hmax, Smin, Smax, Vmin, Vmax pentru calibrare.

Alți parametri: LaserAngle, ResponseDist, etc.

scripts/autopilot_main.py

Rol: Creierul principal al proiectului. Acest script conține clasa LineDetect, care implementează toată logica de detecție, decizie și control. Este nodul central care procesează imaginile de la cameră și trimite comenzi de mișcare robotului.

scripts/autopilot_common.py

Rol: O bibliotecă de funcții și clase ajutătoare, utilizată de autopilot_main.py. Separarea acestor funcții menține codul principal mai curat și mai organizat. Probabil conține clase precum ROSCtrl (pentru a interacționa cu ROS) și color_follow (pentru algoritmii de procesare a imaginii).

scripts/HSV.yaml

Rol: Bază de date pentru calibrarea culorilor. După ce culorile sunt calibrate în mod interactiv, valorile HSV optime sunt salvate în acest fișier. La fiecare pornire a programului, aceste valori sunt citite, eliminând necesitatea recalibrării constante.

## 3. Logica Detaliată a autopilot_main.py
Clasa LineDetect este inima aplicației.

__init__(self) - Inițializarea
La pornire, se configurează starea inițială a robotului:

Inițializare ROS: Se pornește nodul LineDetect.

Stări Inițiale: Se definesc variabile de stare, cum ar fi Track_state = 'identify' (robotul este în așteptare), Calibration = False, is_searching = False.

Definirea Traseelor: Se stabilește lista de culori pe care robotul le poate urmări: self.path_colors = ['red', 'yellow'].

Definirea Țintelor: Se inițializează două variabile esențiale pentru logica de comutare:

self.current_target_color = 'red': Culoarea pe care robotul o urmărește activ în prezent.

self.intended_target_color = 'red': Culoarea pe care utilizatorul intenționează ca robotul să o urmeze. Inițial, sunt identice.

Încărcare Configurații: Se citesc valorile HSV din HSV.yaml pentru toate culorile cunoscute (red, green, blue, yellow).

Configurare PID: Se inițializează controlerul PID cu valorile Kp, Ki, Kd definite.

Pornire Server Dynamic Reconfigure: Permite modificarea parametrilor din rqt_reconfigure.

process(self, rgb_img, action) - Bucla Principală de Procesare
Această metodă este apelată pentru fiecare cadru video primit de la cameră.

Procesare Comenzi: Se verifică tastele apăsate de utilizator (space pentru a porni, c pentru calibrare, r pentru reset, și s pentru a schimba intenția de traseu).

Detecție Multi-Culoare: Dacă robotul nu este în modul calibrare, el caută simultan ambele linii (red și yellow). Acest lucru este realizat eficient prin crearea unui fir de execuție (thread) separat pentru fiecare culoare. Astfel, robotul știe în permanență dacă una sau ambele linii sunt în câmpul său vizual.

Logica de Urmărire și Comutare: Aceasta este cea mai complexă parte:

Se verifică mai întâi dacă linia intenționată (intended_target_color) este vizibilă.

Dacă DA, și dacă aceasta nu este deja linia curentă, se face comutarea: current_target_color devine egal cu intended_target_color, se afișează mesajul "Am gasit linia [...], am comutat!", și se apelează execute() pentru a urmări noua linie.

Dacă linia intenționată NU este vizibilă, se verifică dacă linia curentă (current_target_color) mai este vizibilă.

Dacă DA, se continuă urmărirea ei normal, apelând execute().

Dacă NICIUNA dintre linii nu este vizibilă (sau cel puțin nu cea curentă/intenționată), se apelează start_search_routine() pentru a începe căutarea.

execute(self, circle) - Execuția Mișcării
Rol: Calculează și trimite comanda de mișcare atunci când o linie este detectată.

Calculul Erorii: Calculează eroarea ca fiind diferența dintre centrul liniei detectate și centrul imaginii (self.last_known_error = circle[0] - 320). Această eroare este stocată pentru a fi folosită de rutina de căutare.

Comanda PID: Eroarea este trimisă controlerului PID, care returnează o viteză unghiulară (z) necesară pentru a corecta traiectoria.

Publicare Viteză: Se publică viteza liniară constantă (self.linear) și viteza unghiulară calculată (z) către robot.

start_search_routine(self) - Rutina de Căutare
Rol: Se activează când robotul pierde linia.

Decizia de Rotire: Folosește ultima eroare cunoscută (self.last_known_error) pentru a decide direcția de rotație.

Dacă eroarea a fost pozitivă (linia era în dreapta), robotul se va roti spre dreapta.

Dacă eroarea a fost negativă (linia era în stânga), robotul se va roti spre stânga.

Mișcarea: Robotul se oprește din avansat (linear = 0) și începe să se rotească pe loc până când re-detectează linia.

## 4. Mecanismul de Schimbare a Liniei (Tasta 'S') - Explicație Detaliată
Aceasta este una dintre cele mai ingenioase funcționalități ale proiectului. Comutarea nu este instantanee, ci bazată pe un mecanism "intenție-confirmare".

Pasul 1: Semnalarea Intenției (Apăsarea tastei 'S')

Când utilizatorul apasă tasta 'S', singurul lucru care se schimbă este variabila self.intended_target_color. Aceasta comută între 'red' și 'yellow'.

Robotul nu își schimbă imediat comportamentul. El continuă să urmărească self.current_target_color, dar acum are o nouă "misiune" în fundal: să găsească linia intended_target_color.

Pasul 2: Căutarea și Confirmarea Vizuală

Datorită detecției multi-threaded, robotul continuă să proceseze imaginea pentru ambele culori, chiar dacă urmărește activ doar una.

Bucla process() verifică la fiecare cadru dacă noua culoare intenționată a apărut în câmpul vizual.

Pasul 3: Execuția Comutării

În momentul în care noua linie intenționată este detectată (is_intended_detected devine True), logica de comutare se activează.

Variabila self.current_target_color este actualizată pentru a fi egală cu self.intended_target_color.

Din acest moment, controlerul PID va folosi coordonatele noii linii pentru a calcula eroarea, iar robotul își schimbă efectiv traseul.

Acest mecanism previne comportamentul haotic. Robotul nu va încerca să sară pe un alt traseu "în orb", ci va face trecerea doar atunci când are o confirmare vizuală clară că noul traseu este disponibil și detectabil.
