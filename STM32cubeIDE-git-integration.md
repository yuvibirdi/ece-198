# About EGit
I used EGit to manipulate the repository directly from STM32cubeIDE. It should work with MacOS: I (Vincent) am using Windows.<br /><br />
I added my own STM32 project "earthquake-slave" since STM32cubeIDE was not recognizing the "slave" and "master" projects cloned from GitHub as source code and refused to build anything in it. Currently, EGit is the only way I know how to use GitHub with STM32 programming. <br /><br />
VSCode was used to add markdown files (I don't think EGit can do that). I will also refer back to this doc in upper years if I need to do MCU programming again. <br />

# How to install and use EGit
- Open STM32cubeIDE
- Top bar: Help
- Eclipse Marketplace
- Search “egit”
- Download “EGit - Git Integration for 6.7.0” (version may vary)
- Wait for install to finish (progress bar in bottom right corner)
- Restart IDE
- EGit is ready to use

Video demonstrating the above: [Install Git support in STM32CubeIDE](https://www.youtube.com/watch?v=dCE-4dgL82o)

Video on how to use EGit: [Git in STM32CubeIDE: Import, Edit, and Push to GitHub Explained](https://www.youtube.com/watch?v=EMME859o5u0) 

- This tutorial tells you everything you need to know about cloning, committing, and adding new projects to the repository.

# When IDE asks for GitHub sign-in

USER = GitHub username

PASS = Personal Access Token (PAT) (or enter GitHub account password if PAT doesn’t work for you)

NOTE: IDE may ask you to sign in twice in a row for no good reason. Just enter the same USER/PASS information again and again.

# Generate PAT

[github.com](http://github.com) settings → developer → Classic token → Classic token → select repo (should be enough for our purposes)

Paste your PAT in your notes app or wherever you want to keep it private so you can paste it into IDE later.
