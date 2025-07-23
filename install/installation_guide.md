# 🚀 PyChrono Installation Guide

This guide walks you through the installation of [PyChrono](https://projectchrono.org/) for use in this project.

> [!NOTE]  
> Highlights information that users should take into account, even when skimming.

> [!TIP]
> Optional information to help a user be more successful.

> [!IMPORTANT]  
> Crucial information necessary for users to succeed.

> [!WARNING]  
> Critical content demanding immediate user attention due to potential risks.

> [!CAUTION]
> Negative potential consequences of an action.

---

## 🐍 1. Install Anaconda or Miniconda

Follow the official [Miniconda installation guide](https://www.anaconda.com/docs/getting-started/miniconda/install#macos-linux-installation) for your platform (Windows, macOS, or Linux).

---

## ⚙️ 2. Automated Setup

### ✅ Windows Users

1. Download the file `install_pychrono_windows.txt`.
2. Rename it to `install_pychrono_windows.bat`.
3. Open **Command Prompt** and run:

   ```bash
   install_pychrono_windows.bat
   ```

4. ➡️ [Skip to Step 5: Copy the Demos and Test the Installation](#📁-5-copy-the-demos-and-test-the-installation)

---

### ✅ Linux Users

1. Download the file `install_pychrono_linux.sh`.
2. Open a terminal and make the script executable:

   ```bash
   chmod +x install_pychrono_linux.sh
   ```

3. Run the script:

   ```bash
   ./install_pychrono_linux.sh
   ```

4. ➡️ [Skip to Step 6: Run the Code](#🚦-6-run-the-code)

---

### ✅ MacOS Users

1. Download the file `install_pychrono_macos.sh`.
2. Open a terminal and make the script executable:

   ```bash
   chmod +x install_pychrono_macos.sh
   ```

3. Run the script:

   ```bash
   ./install_pychrono_macos.sh
   ```

4. Initialize Conda for zsh shell
   ```bash
   /opt/miniconda3/bin/conda init zsh
   ```

5. Restart the shell
   ```bash
   exec zsh
   ```

6. Activate the PyChrono Conda environment
   ```bash
   conda activate chrono
   ```

7. Set the PYTHONPATH environment variable
   ``` bash
   export PYTHONPATH=$(conda info --base)/envs/chrono/share/chrono/python
   ```

8. ➡️ [Skip to Step 6: Run the Code](#🚦-6-run-the-code)

> [!NOTE]  
> Steps 4-7 need to be run every time you want to use pychrono.

---

## 🛠 3. Manual Setup Procedure (All Platforms)

### 🔧 Step 1: Add the `conda-forge` channel

Open a terminal (Windows users: use PowerShell or Anaconda Prompt):

```bash
conda config --add channels http://conda.anaconda.org/conda-forge
```

---

### 🔧 Step 2: Create a new Conda environment

```bash
conda create -n chrono python=3.10
```

Activate the environment:

```bash
conda activate chrono
```

To deactivate:

```bash
conda deactivate
```

**Stay in the environment** during the following steps.

---

### 🔧 Step 3: Install PyChrono dependencies

```bash
conda install -c conda-forge numpy=1.24.0
conda install -c conda-forge matplotlib
conda install -c conda-forge irrlicht=1.8.5
conda install -c conda-forge pytz
conda install -c conda-forge scipy
```

---

### 🔧 Step 4: Download and install PyChrono package

Visit the [package repo](https://anaconda.org/projectchrono/pychrono/files?page=3) and download the correct version for your platform:

- **macOS**: `osx-arm64/pychrono-8.0.0-py310_2471.tar.bz2`
- **Windows**: `win-64/pychrono-8.0.0-py310_0.tar.bz2`
- **Linux**: `linux-64/pychrono-8.0.0-py310_0.tar.bz2`

After downloading, navigate to the folder (e.g., Downloads) and run:

```bash
cd ~/Downloads
conda install pychrono-8.0.0-py310_2471.tar.bz2
```

> 🧠 **Note (macOS users)**: You may also need to set the `PYTHONPATH`:

```bash
export PYTHONPATH=$HOME/miniconda3/envs/chrono/share/chrono/python
```

---

## 📁 5. Copy the Demos and Test the Installation

1. Navigate to the folder where you want to save the demo files:

   ```bash
   mkdir ~/pychrono_demos
   cd ~/pychrono_demos
   ```

2. Copy the demo files:

   ```bash
   cp -r $PYTHONPATH/pychrono/demos/* .
   ```

3. Run a demo to verify the installation:

   ```bash
   cd mbs
   python demo_MBS_revolute.py
   ```

---

## 🚦 6. Run the Code

1. Clone the repository:

   ```bash
   git clone https://github.com/andrealaffly/UAV_Sim_PyChrono.git
   cd UAV_Sim_PyChrono
   ```

2. Run the simulation:

   ```bash
   python main.py
   ```

---
