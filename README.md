# Embedded Systems Projekt
## Fractals und Langtons Ant

Ziel des Projekts war es ursprünglich Lnagtons Ant mithilfe des Displays(TI Educational Booster Pack MKII) darzustellen.
Nach dem Anfang des Projekts entpuppte sich dies schwerer als gedacht. Für die Konfiguration TM4C1294NCPDT in Kombination mit dem Booster Pack war online keine library zu finden(die Verwendung von Energia war untersagt, da zu einfach).
Mithilfe eines Oszilloskopes und vielen schlaflosen Nächten war es letztenendes möglich mithilfe von Schnippseln aus anderen Librarys das Display zum laufen zu kriegen. Während dieses Projektes habe ich viele Dinge über die Programmierung in C gelernt. Da die meisten anderen Libraries in C++ geschrieben waren oder Energia spezifische Funktionen verwendet haben musste ich diese in C nachprogrammieren. An einigen Stellen kam sogar Assemblersprache vor.
Da mir Langtons Ant am Ende zu eintönig schien habe ich zusätzlich einen Fractal Modus und weitere Peripherien(Buzzer, Buttons, Joystick) zur Interaktion implementiert.

Weitere Details und eine Anleitung sind in der main.c angeführt

Unter den Files befindet sich außerdem ein Demo Video(demo.mp4)
