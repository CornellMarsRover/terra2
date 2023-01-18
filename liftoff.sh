#!/bin/bash
# The liftoff script will create a CMR development directory and bootstrap it with the entire
# codebase from each code repository. It will then install the command line interface and download
# CMR Docker images.

# Cute little introduction.
echo "🚀 🚀 🚀 🚀"
echo "Hi, I'm Terraformer :) I'll do my best to set up your machine for rover development."

if [[ -z $PKG_MAN ]]; then
    # If the user didn't specify a package manager, try to detect it based on the OS.
    # terraformer only supports Ubuntu (including on Windows via WSL) and macOS, so only check for those.
    if [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS
        PKG_MAN="brew"
    elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
        # Linux
        PKG_MAN="apt-get"
    elif [[ "$OSTYPE" == "cygwin" ]]; then
        # Windows with emulated Linux environment and POSIX compatibility layer
        PKG_MAN="apt-get"
    elif [[ "$OSTYPE" == "msys" ]]; then
        # MinGW (i.e. Windows)
        PKG_MAN="apt-get"
    else
        # Unsupported OS
        echo "Your detected operating system ($OSTYPE) is not supported."
        return 1
    fi
    echo "Since \$PKG_MAN was not set, I'll use the auto-detected package manager: $PKG_MAN"
fi

# Make sure the package manager actually works.
eval $PKG_MAN &> /dev/null
if [[ $? == 127 ]]; then
    echo "$PKG_MAN was not found. Aborting."
    return 1
fi

# Make sure git is installed, and try to install it if not.
command -v git >/dev/null 2>&1 ||
{ echo "Git is not installed. Attempting to install...";
  eval "$PKG_MAN install git"
  if [[ $? -ne 0 ]]; then
    echo "Failed to install Git using $PKG_MAN. Aborting."
    return 1
  fi
}

# Make sure xpra is installed, and try to install it if not.
command -v xpra >/dev/null 2>&1 ||
{ echo "!!!! ERROR !!!!"
  echo "Xpra is not installed."
  echo "I can't install it for you because the setup process varies for different systems."
  echo "See this documentation article for steps: https://github.com/Xpra-org/xpra/wiki/Download"
  return 1
}

# Make sure Docker is installed, and tell the user to install it if not.
# terraformer won't try to install Docker itself because the installation is often not trivial.
command -v docker >/dev/null 2>&1 ||
{ echo "!!!! ERROR !!!!"
  echo "Docker is not installed."
  echo "I can't install it for you because the setup process varies for different systems."
  # TODO: Update this documentation link to the Docker installation page once it's created.
  echo "See this documentation article for steps: https://docs.cornellmarsrover.org/"
  return 1
}

# Make sure Python 3 is installed and try to install it if not.
command -v python3 >/dev/null 2>&1 ||
{ echo "Python 3 is not installed. Attempting to install..."
  eval "$PKG_MAN install python3"
  if [[ $? -ne 0 ]]; then
    echo "Failed to install Python 3 using $PKG_MAN. Aborting."
    return 1
  fi
}

# Make sure Pip is installed and try to install it if not
command -v pip >/dev/null 2>&1 ||
{ echo "Pip is not installed. Attempting to install..."
  eval "$PKG_MAN install pip"
  if [[ $? -ne 0 ]]; then
    echo "Failed to install Pip using $PKG_MAN. Aborting."
    return 1
 fi
}

# Make sure we have our SSH key set up with GitHub
ssh -T git@github.com &> /dev/null
SSH_KEY_WORKS=$?
if [ $SSH_KEY_WORKS -ne 1 ]; then
    echo "Adding SSH key for github"
    if [ ! -f ~/.ssh/id_rsa ]; then
        read -p "Enter github email: " email
        echo "Using email $email"
        eval `ssh-agent`
        ssh-keygen -t rsa -b 4096 -C $email
        ssh-add ~/.ssh/id_rsa
    fi
    curl -fsSL https://raw.githubusercontent.com/CornellMarsRover/liftoff/main/github-keygen.py -o github-keygen.py
    pub=`cat ~/.ssh/id_rsa.pub`
    pip install requests
    python3 github-keygen.py "$pub"
    KEY_ADD_SUCCESS=$?
    if [ $KEY_ADD_SUCCESS == 0 ]; then
        echo "Successfully added ssh key"
    else
        return $KEY_ADD_SUCCESS
    fi
fi

# Check if CMR_ROOT is set and set to its default if not.
if [[ -z $CMR_ROOT ]]; then
    export CMR_ROOT=$HOME/cmr
    PATH_HELP=1
fi

# Create the CMR_ROOT directory if it doesn't already exist.
mkdir -p $CMR_ROOT

# Move to the CMR_ROOT directory
pushd $CMR_ROOT &> /dev/null

# Clone repositories

echo ""
echo "Cloning CMR repositories into $CMR_ROOT:"

for name in terraformer terra phobos-gui phobos-cli micro playground;
    do
        if [ -d "$CMR_ROOT/$name" ]; then echo "You already have $name, skipping..."
        else 
            echo "Cloning $name from remote..."
            echo "(start git output)"
            git clone "git@github.com:CornellMarsRover/$name.git"
            echo "(end git output)"
            echo "Done cloning $name from remote."
        fi
    done

# Pull down cornellmarsrover/dev:latest

echo ""
echo "Pulling down the dev image (latest)..."
docker pull cornellmarsrover/dev:latest

# Install Phobos CLI from terraformer's bundled wheel.

# Make sure the repository is latest.
pushd $CMR_ROOT/terraformer &> /dev/null
git pull
popd &> /dev/null

# Use "find" to find the wheel file included in terraformer.
CLI_WHEEL_PATH=$(find terraformer/dist -name "*.whl")
CLI_WHEEL_PATH="$CMR_ROOT/$CLI_WHEEL_PATH"
# Get the CLI version using "awk", which looks for the version number between two dashes (-).
# We only do this for the user's sake, so that they know which version they're getting.
CLI_VERSION=$(basename $CLI_WHEEL_PATH | awk -F '-|-' '{print $2}')
echo ""
echo "Installing Phobos CLI v$CLI_VERSION..."
echo "(start pip output)"
python3 -m pip install --disable-pip-version-check --force-reinstall "$CLI_WHEEL_PATH"
if [[ $? -ne 0 ]]; then
    echo "Failed to install Phobos CLI. Aborting."
    return 1
fi
echo "(end pip output)"
echo "Done installing Phobos CLI v$CLI_VERSION."

# Install the git hook tool pre-commit
command -v pre-commit >/dev/null 2>&1 ||
{ echo "Pre-commit is not installed. Attempting to install..."
  eval "pip install pre-commit"
  if [[ $? -ne 0 ]]; then
    echo "Failed to pip install pre-commit. Aborting"
    return 1
 fi

 # Install the hooks
 pushd "$CMR_ROOT/terra" &> /dev/null
 pre-commit install
 pre-commit install --hook-type pre-push
 popd &> /dev/null
}

if [[ ! -z $PATH_HELP ]]; then
    echo ""
    echo "!!!! IMPORTANT !!!!"
    echo "I created the CMR root directory in your home folder for you."
    echo "You must manually set the \$CMR_ROOT environment variable in your shell profile by appending this line to it:"
    echo ""
    echo "export CMR_ROOT=$CMR_ROOT"
    echo ""
fi

echo "We have liftoff! Successfully bootstrapped your machine for rover development."

# Go back to whatever directory the user called from (undoes the pushd instruction from earlier)
popd &> /dev/null
