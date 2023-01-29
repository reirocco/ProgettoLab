
<h1 align="center">
  <br>
  <a href="#"><img src="https://github.com/reirocco/ProgettoLab/blob/main/RESOURCES/logo.png" alt="Markdownify" width="300"></a>
  <br>
  BallBot BB1
  <br>
</h1>

<h4 align="center"> Laboratorio di Automazione 2022/23 <a href="https://www.univpm.it/Entra/" target="_blank">@UNIVPM</a>.</h4>
<h5 align="center"> <a href="https://www.overleaf.com/read/gyjkqwsypvry">Clicca qui</a> per vedere l'Overleaf del progetto</h5>


<p align="center">
  <a href="#short-description">Short Description</a> •
  <a href="#how-to-use">How To Use</a> •
  <a href="#download">Download</a> •
</p>



## Short Description

Il BallBot è un robot mobile dinamicamente stabile progettato per bilanciarsi su una singola ruota sferica (ovvero una palla). Attraverso il suo unico punto di contatto con il suolo, un ballbot è omnidirezionale e quindi eccezionalmente agile, manovrabile e organico in movimento rispetto ad altri veicoli terrestri. La sua stabilità dinamica consente una migliore navigabilità in ambienti stretti, affollati e dinamici. Il ballbot funziona secondo lo stesso principio di un pendolo invertito.

## How To Use

La repository include tutto il materiale utilizzato per lo svolgimento del laboratorio e la sua struttura si divide in:
* Library --> contiene tutte le librerie scritte ad hoc per il corretto funzionamento dei componenti hardware
* Materiale --> tutto il materiale trovato in letteratura che è servito per capire il funzionamento delle parti e per l'inizio del progetto (Datasheet dei componenti compresi).
* MATLAB --> script utilizzati per il testing dei componenti via Seriale e per la visualizzazione grafica in "real-time" dei valori dei sensori.
* Note --> appunti per i calcoli
* STL --> file stampati in 3d
* STM_CUBE_MX --> WorkSpace di tutti i progetti dell'IDE
* DYNAMIC_BALLBOT --> Progetto finale con l'implementazione del sistema di controllo

```bash
# Clone this repository
$ git clone https://github.com/reirocco/ProgettoLab.git

# Go into the repository
$ cd ProgettoLab

```

> **Note**
> If you're using Linux Bash for Windows, [see this guide](https://www.howtogeek.com/261575/how-to-run-graphical-linux-desktop-applications-from-windows-10s-bash-shell/) or use `node` from the command prompt.


## Download
Software utilizzati:
* [STM-CUBE-MX IDE](https://www.st.com/en/development-tools/stm32cubeide.html)
* [Arduino IDE](https://www.arduino.cc/en/software)
* [MarkDown Monster](https://markdownmonster.west-wind.com/download)
* [MATLAB](https://it.mathworks.com/products/matlab.html)
* [Tauno Serial Plotter](https://github.com/taunoe/tauno-serial-plotter)
