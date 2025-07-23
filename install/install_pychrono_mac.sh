#!/bin/bash
# Edison Melendez
# — Modified by Giri Mugundan Kumar (7/23/25)
# NOTES: 1. Added code for ToS acceptance while adding a channel in conda.
#        2. Prompt and download of the correct pychrono tarball package added.
#        3. Much more simplified by not using the home folder but the Downloads 
#           folder for all the installation activites.

set -e
echo "==== PyChrono Installer for macOS (M1/M2/M3) ==== name date"

# === Step 0: Auto-detect Miniconda install path ===
if [ -x "/opt/miniconda3/bin/conda" ]; then
    MINICONDA_PATH="/opt/miniconda3"
elif [ -x "$HOME/miniconda3/bin/conda" ]; then
    MINICONDA_PATH="$HOME/miniconda3"
else
    echo "❌ Miniconda not found in /opt/miniconda3 or $HOME/miniconda3"
    echo "   ➡️ Please install Miniconda from: https://docs.conda.io/en/latest/miniconda.html"
    exit 1
fi

export PATH="$MINICONDA_PATH/bin:$PATH"

# === Step 1: Check conda ===
if ! command -v conda &> /dev/null; then
    echo "❌ Conda not found in PATH. Try restarting your terminal or check PATH settings."
    exit 1
fi

# === Step 1.5: Ensure conda-forge channel is added ===
echo "➡️  Adding conda-forge channel (if not already added)..."
conda config --add channels conda-forge || true  # harmless if already added

# === Step 1.6: Accept Anaconda Terms of Service (TOS) ===
echo "➡️  Ensuring Anaconda Terms of Service are accepted for required channels..."
CHANNELS=(
    "https://repo.anaconda.com/pkgs/main"
    "https://repo.anaconda.com/pkgs/r"
)
for CHANNEL in "${CHANNELS[@]}"; do
    echo "   🔎 Accepting ToS for: $CHANNEL"
    conda tos accept --override-channels --channel "$CHANNEL" || {
        echo "❌ Failed to accept ToS for $CHANNEL"
        echo "   💡 Please manually run:"
        echo "      conda tos accept --override-channels --channel $CHANNEL"
        exit 1
    }
done

# === Step 2: Check or create 'chrono' environment ===
if ! conda info --envs | grep -q "^chrono"; then
    echo "⚠️  Conda environment 'chrono' not found."
    read -p "➡️  Do you want to create it now with Python 3.10? [Y/n]: " CREATE_ENV
    if [[ "$CREATE_ENV" =~ ^[Nn]$ ]]; then
        echo "❌ Aborting setup. Please create it manually: conda create -n chrono python=3.10"
        exit 1
    else
        echo "➡️  Creating conda environment 'chrono'..."
        conda create -y -n chrono python=3.10
    fi
fi

# === Step 3: Activate environment ===
echo "➡️  Activating conda environment 'chrono'"
source "$MINICONDA_PATH/etc/profile.d/conda.sh"
conda activate chrono

# === Step 4: Install required packages ===
echo "➡️  Installing required packages..."
conda install -y -c conda-forge numpy=1.24.0
conda install -y -c conda-forge matplotlib
conda install -y -c conda-forge irrlicht=1.8.5
conda install -y -c conda-forge pytz
conda install -y -c conda-forge scipy

=== Step 5: Ensure PyChrono tarball is available ===
TARBALL="$HOME/Downloads/pychrono-8.0.0-py310_2471.tar.bz2"
TARBALL_URL="https://anaconda.org/projectchrono/pychrono/8.0.0/download/osx-arm64/pychrono-8.0.0-py310_2471.tar.bz2"

if [ ! -f "$TARBALL" ]; then
    echo "⚠️  PyChrono tarball not found at: $TARBALL"
    read -p "➡️  Do you want to download it now from projectchrono [Y/n]?: " DOWNLOAD_PROMPT
    if [[ "$DOWNLOAD_PROMPT" =~ ^[Nn]$ ]]; then
        echo "❌ Aborting. Please download this file manually and place it in your Downloads folder:"
        echo "   $TARBALL_URL"
        exit 1
    fi

    echo "⬇️  Downloading PyChrono tarball..."
    curl -L -o "$TARBALL" "$TARBALL_URL" || {
        echo "❌ Download failed. Please try downloading manually:"
        echo "   $TARBALL_URL"
        exit 1
    }
    echo "✅ Download complete: $TARBALL"
fi

# === Step 6: Install PyChrono ===
echo "➡️  Installing PyChrono from tarball..."
conda install "$TARBALL"


# === Step 7: Set PYTHONPATH ===
export PYTHONPATH="$MINICONDA_PATH/envs/chrono/share/chrono/python"
echo "✅ PYTHONPATH set to: $PYTHONPATH"

# === Step 8: Prompt to delete tarball ===
read -p "🧹 Do you want to delete the downloaded PyChrono tarball? [y/N]: " DELETE_TARBALL
if [[ "$DELETE_TARBALL" =~ ^[Yy]$ ]]; then
    rm -f "$TARBALL" && echo "✅ Deleted: $TARBALL"
else
    echo "🗂  PyChrono tarball kept at: $TARBALL"
fi

echo "✅ PyChrono installation complete!"