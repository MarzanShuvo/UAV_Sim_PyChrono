#!/bin/bash
# Created: 7/20/2025 by Edison Melendez
# Modified: 7/22/2025 by Mattia Gramuglia

echo "==== Installing PyChrono on Linux ===="

# Step 0: Check and install Miniconda if needed
if ! command -v conda &> /dev/null; then
  echo "⚠️  Conda not found. Installing Miniconda..."

  MINICONDA_INSTALLER=Miniconda3-latest-Linux-x86_64.sh
  wget https://repo.anaconda.com/miniconda/$MINICONDA_INSTALLER -O /tmp/$MINICONDA_INSTALLER

  bash /tmp/$MINICONDA_INSTALLER -b -p $HOME/miniconda3
  rm /tmp/$MINICONDA_INSTALLER

  # Initialize conda
  eval "$($HOME/miniconda3/bin/conda shell.bash hook)"
  conda init
  echo "✅ Miniconda installed successfully. Please restart your shell before running this script again."
  exit 0
else
  echo "✅ Conda already installed."
  source "$(conda info --base)/etc/profile.d/conda.sh"
fi

# Step 1: Accept Terms of Service for required channels
echo "⚠️  Accepting Terms of Service for conda default channels..."
conda tos accept --channel https://repo.anaconda.com/pkgs/main || true
conda tos accept --channel https://repo.anaconda.com/pkgs/r || true

# Step 2: Add conda-forge channel
echo "🔧 Adding conda-forge channel..."
conda config --add channels conda-forge
conda config --set channel_priority strict

# Step 3: Create conda environment if not already created
if conda env list | grep -q "^chrono\s"; then
  echo "✅ Conda environment 'chrono' already exists."
else
  echo "📦 Creating conda environment 'chrono' with Python 3.10..."
  conda create -y -n chrono python=3.10
fi

# Step 4: Activate environment
echo "⚙️  Activating environment..."
conda activate chrono

# Step 5: Install dependencies
echo "📦 Installing required packages..."
conda install -y -c conda-forge numpy=1.24.0 matplotlib irrlicht=1.8.5 pytz scipy

# Step 6: Install PyChrono from tarball
VERSION="8.0.0"
PYTHON_TAG="py310_0"
FILENAME="pychrono-${VERSION}-${PYTHON_TAG}.tar.bz2"
DOWNLOAD_URL="https://anaconda.org/projectchrono/pychrono/${VERSION}/download/linux-64/${FILENAME}"
DOWNLOAD_DIR="$HOME/Downloads"
TARBALL="$DOWNLOAD_DIR/$FILENAME"

echo "📂 Checking for PyChrono tarball at: $TARBALL"
if [ ! -f "$TARBALL" ]; then
  echo "⬇️  Downloading PyChrono $VERSION tarball..."
  wget -O "$TARBALL" "$DOWNLOAD_URL"
else
  echo "✅ PyChrono tarball already exists."
fi

echo "📦 Installing PyChrono from tarball..."
conda install -y "$TARBALL"

# Step 7: Set PYTHONPATH
PYTHONPATH=$(conda info --base)/envs/chrono/share/chrono/python
export PYTHONPATH
echo "✅ PYTHONPATH set to: $PYTHONPATH"

# Step 8: Ensure PYTHONPATH is added to .bashrc if not already present
BASHRC="$HOME/.bashrc"
PYTHONPATH_LINE="export PYTHONPATH=$PYTHONPATH"

if grep -Fxq "$PYTHONPATH_LINE" "$BASHRC"; then
  echo "✅ PYTHONPATH already set in $BASHRC"
else
  echo "$PYTHONPATH_LINE" >> "$BASHRC"
  echo "✅ PYTHONPATH added to $BASHRC"
fi

# Step 9: Suggest testing demos
DEMO_PATH=$(python -c "import pychrono; import os; print(os.path.dirname(pychrono.__file__) + '/demos')")
echo "✅ PyChrono installation complete. You can now test a demo:"
echo "Example:"
echo "    conda activate chrono"
echo "    mkdir ~/pychrono_demos && cd ~/pychrono_demos"
echo "    cp -r $DEMO_PATH/* ."
echo "    cd mbs && python demo_MBS_revolute.py"

