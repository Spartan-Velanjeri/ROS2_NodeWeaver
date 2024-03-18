# Description

- Download the maps from the Artifactory using `curl` and add them to the build folder
- Maps will be installed after download, so they can be found the `lu_rough_localization` package's `share` folder

# Credentials setup

Credentials for the JFrog Artifactory are necessary for this, to set up the credentials now, there are a couple of options. First you need to go to [this page](https://artifactory.boschdevcloud.com/ui/user_profile) and click on `Generate an Identity Token`. Copy the token locally the time being.

## Setup credentials with `keyring`

* Install `keyring`: `sudo pip3 install keyring`
* Set your username for the Artifactory account as `keyring set btr-jfrog-user $(whoami)` and enter your username
* Set your token for the Artfifactory accoung as `keyring set btr-jfrog-token $(whoami)` and enter the token you generated
* Build the workspace, it should retrieve the credentials using `keyring`

## Setup credentials as environment variables (**unsafe**)

```bash
echo "export BTR_JFROG_USER=<YOUR_USERNAME>" >> ~/.bashrc
echo "export BTR_JFROG_TOKEN=<YOUR_TOKEN>" >> ~/.bashrc
source ~/.bashrc
cd <WORKSPACE>
colcon build
```
