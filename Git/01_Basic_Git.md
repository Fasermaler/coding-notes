# 01 Basic Git 

Author: Fasermaler

This guide will cover the basics of the Git workflow and how to use Github - a code hosting platform based on Git.

## Pre-requisites

- Basic bash knowledge
- Interest in learning how to *git gud*

## Introduction

Git is a version control framework that is widely used. You might have heard of GitHub - a thriving open source community where people share code and work together to improve each other's work. GitHub uses Git, **but Git does not actually require GitHub**. 

One of the big advantages of Git is that it is a Distributed Version Control System (DVCS). This means that instead of having a single computer / location that stores all the version control information, each computer working on a project can have their own full workspace that contains a working version of the code as well as all the changes (as long as they remember to fetch them).

Git is also optimized, fast and secure. Because of this, it has risen quickly to be adopted as the standard for version control - especially open-source version control. Git is quintessential as a software developer and learning it will augment your projects, big or small. 

One more thing to note: As you go through this guide, feel free to create a test repository to try out the various concepts and instructions. Experience with Git does not come by reading about it, it comes from actually **using Git as part and parcel of daily software development.**

In this first part of the guide, we will use GitHub as it requires no command line, no special tools or what-so-ever to get started. In later guides, we will cover GitKraken and the Git Command Line.

## GitHub

GitHub is a code hosting platform that allows users to collaboratively work on large open-source projects as well as do version control on their projects. It is based on Git but does not require Git command line knowledge to use. 

### Hello World

The following section is based heavily on the GitHub Introductory [Guide](https://guides.github.com/activities/hello-world/#intro).

The easiest way to learn GitHub (and Git) is by doing. This section aims to provide a hands on guide on how to conduct basic operations in GitHub. You will need:

- [GitHub.com Account](https://github.com/)
- Internet Access

You do not need to know how to code or use a command line.

#### Creating a Repository

The first step is to create a repository. A repository is a working directory or location where project code, assets and resources are organized. In most cases, a repository will also include:

- README: A file to explain the project and how to use it to newcomers (or perhaps yourself in the future!)
- License: A license to advise others on how they can use or incorporate your project into other projects
- GitIgnore: A file that tells Git which files should be ignored from being tracked (sensitive files like private keys and etc)

GitHub makes it easy to add the first two when initializing a repository. Begin by going to [GitHub.com](https://github.com/) and logging in.

1. In the upper right corner, next to your avatar or identicon, click `+` and then select **New repository**.
2. Name your repository whatever you'd like, in this case we will go with `hello-world`.
3. Write a short description.
4. Select **Initialize this repository with a README**.

![new-repo-form](assets/create-new-repo.png)

5. Click **Create repository**.

#### Create a Branch

Branching is how multiple versions of the same code can be worked on at the same time. 

By default your repository has one branch named `master` which is considered to be the definitive branch. We use branches to experiment and make edits before committing them to `master`.

When you create a branch off the `master` branch, you’re making a copy, or snapshot, of `master` as it was at that point in time. If someone else made changes to the `master` branch while you were working on your branch, you could pull in those updates.

This diagram shows:

- The `master` branch

- A new branch called `feature` (because we’re doing ‘feature work’ on this branch)

- The journey that `feature` takes before it’s merged into `master`

  ![a branch](assets/branching.png)

Have you ever saved different versions of a file? Something like:

- `story.txt`
- `story-joe-edit.txt`
- `story-joe-edit-reviewed.txt`

Branches accomplish similar goals in GitHub repositories. Except that branches can have their changes compared and merged back into each other seamlessly unlike manual file versioning.

How branches are used properly in a Git workflow will be covered later. In any case, let's get to creating a new branch in `hello-world`:

1. Go to your new repository `hello-world`.

2. Click the drop down at the top of the file list that says **branch: master**.

3. Type a branch name, `readme-edits`, into the new branch text box.

4. Select the blue **Create branch** box or hit “Enter” on your keyboard.

   ![branch gif](assets/readme-edits.gif)

Now you have two branches, `master` and `readme-edits`. They look exactly the same, but not for long! Next we’ll add our changes to the new branch.

#### Making and Committing Changes

So now you are wondering - how do you actually change anything in GitHub? Now, you’re on the code view for your `readme-edits` branch, which is a copy of `master`. Let’s make some edits.

On GitHub, saved changes are called *commits*. Each commit has an associated *commit message*,
which is a description explaining why a particular change was made. Commit messages capture the history of your changes, so other contributors can understand what you’ve done and why.

Guidelines on how to write proper commit messages will be covered later.

For now let's commit a simple change to the README file:

1. Click the `README.md` file.

2. Click the pencil icon in the upper right corner of the file view to edit.

3. In the editor, write a bit about yourself.

4. Write a commit message that describes your changes.

5. Click **Commit changes** button.

![commit](assets/commit.png)

These changes will be made to just the README file on your `readme-edits` branch, so now this branch contains content that’s different from `master`.

If you return to the repository's main page and switch branches back to `master`, you will find that your newly made changes for README would have vanished. Fret not - it's all on the `readme-edits` branch and will be there once you switch back.

#### Open a Pull Request

So now let's say you'd like those spiffy new changes in `readme-edits` to be reflected back in master. The way this is done is through a *Pull Request*.

Pull Requests are the heart of collaboration on GitHub. When you open a *pull request*, you’re proposing your changes and requesting that someone review and pull in your contribution and merge them into their branch. Pull  requests show *diffs*, or differences, of the content from both branches. The changes, additions, and subtractions are shown in green and red.

As soon as you make a commit, you can open a pull request and start a discussion, even before the code is finished.

By using GitHub’s [@mention system](https://help.github.com/articles/about-writing-and-formatting-on-github/#text-formatting-toolbar)  in your pull request message, you can ask for feedback from specific  people or teams, whether they’re down the hall or 10 time zones away.

You can even open pull requests in your own repository and merge them  yourself. It’s a great way to learn the GitHub flow before working on larger projects.

Now let's get to work on making a pull request:

1. Click the  **Pull Request** tab, then from the Pull Request page, click the green **New pull request** button.

   ![https://guides.github.com/activities/hello-world/pr-tab.gif](assets/pr-tab.gif)

2. In the **Example Comparisons** box, select the branch you made, `readme-edits`, to compare with `master` (the original).

   ![https://guides.github.com/activities/hello-world/pick-branch.png](assets/pick-branch.png)

3. Look over your changes in the diffs on the Compare page, make sure they’re what you want to submit.

   ![https://guides.github.com/activities/hello-world/diff.png](assets/diff.png)

   In the event if 2 lines have different changes (due to other modifications to `master`), this is also where you will get to pick which changes you would like to keep for that specific request.

4. When you’re satisfied that these are the changes you want to submit, click the big green **Create Pull Request** button.

   ![https://guides.github.com/activities/hello-world/create-pr.png](assets/create-pr.png)

5. Give your pull request a title and write a brief description of your changes. A good description is especially important if you are sending a pull request to someone else's repository - you want to give them a good idea of what you have changed and the reasons why you have done so.

![https://guides.github.com/activities/hello-world/pr-form.png](assets/pr-form.png)

#### Merging a Pull Request

Now you have to review and approve the Pull Request for a merge to take place. Without the merge, a Pull Request remains a Pull Request. In this vein, you can screen other's Pull Requests for your repository before approving any changes.

In this example, we are approving our own Pull Request:

1. Click the green **Merge pull request** button to merge the changes into `master`.

2. Click **Confirm merge**.

3. Go ahead and delete the branch, since its changes have been incorporated, with the **Delete branch** button in the purple box. In production, whether you do this or not is up to specific workflows. But generally if you merge to the production or development branch, it's when a feature has been completed. More on this later.

   ![delete](assets/delete-button.png)

#### Done!

Not so bad wasn't it? Within this short span of time, you have:

- Created an open source repository
- Started and managed a new branch
- Changed a file and committed those changes to GitHub
- Opened and merged a Pull Request

Take a look at your GitHub profile and you’ll see your new [contribution squares](https://help.github.com/articles/viewing-contributions)!

## Git WorkFlow

With some basics under your belt, it's time to understand the *Git Workflow*.

The Git workflow was conceived by [Vincent Driessen](https://nvie.com/posts/a-successful-git-branching-model/) on nvie back in 2010. In it, he proposed a now widely adopted approach to using Git to manage a repository, it's features and generally keeping things sane. Poor usage of Git can be highly destructive, thus before anyone delves into using Git itself it is imperative that they understand how the overall workflow is supposed to work.

Below is what Vincent envisioned when he first drafted up the workflow:

![img](assets/git-model@2x.png)

In the previous image, the entire picture depicts a repository. The individual vertical lines (excluding the timeline) are individual *branches* of the repository.

### Decentralized Version Control

Leveraging on the decentralized nature of Git allows developers more flexibility in dealing with their workflow. However, in Vincent's workflow, Vincent proposed that the *origin* repository as the *ground truth* (AKA production repository). 

On top of each developer simply pushing and pulling from the *origin*, developers can form sub-teams who push and pull from each other. This allows multiple developers to work on a larger feature before pushing incomplete features to the *origin* branch.

You can think of the repository on GitHub as the *origin* repository. If you had downloaded AKA *cloned* the repository into your local computer, that will be the local version of your repository. You will then be pushing changes to the *origin* repository on GitHub and fetching changes as needed.

![img](../../../Consistently%20Updated/coding-notes/Git/assets/centr-decentr@2x.png)

### Main Branches

Vincent proposed two main branches - *master* and *develop* within the *origin* repository. If this sounds confusing to you, this is the hierarchy as visualized in GitKraken:

![1564656014639](../../../Consistently%20Updated/coding-notes/Git/assets/1564656014639.png)

#### Master Branch

The *origin/master* branch is the production ready branch. This branch is reserved only for production-ready versions of the codebase. 

The *master* branch should only be merged to if the changes made are stable enough for production. In fact, by being strict enough such that each commit to the master branch is a production-ready version of the code, it is possible to use a Git hook script to automatically build and roll out each release version. 

#### Develop Branch

Running parallel to the *master* branch is the *origin/develop* branch. This branch is also known as the *integration* branch and is used to reflect the latest developmental changes to the code. This does not mean that broken or incomplete code is to be thrown here willy-nilly. This branch is for latest developments of complete features that might be unstable or require further development. However if someone else were to *clone* this repository they should **still be able to run the code.**

If it's hard to wrap your head around it, an analogy would be a beta branch of a game. The game still runs - it's just unstable. If the game can't even run, that change shouldn't be placed into the beta branch.

|                         | Master                | Develop                       |
| ----------------------- | --------------------- | ----------------------------- |
| Type of merge           | Production-ready code | Latest Developmental Features |
| Who merges?             | Repository Maintainer | Everyone / team leads         |
| Broken Code acceptable? | Hell no               | No                            |

### Supporting Branches

Supporting branches are other smaller branches that make it easier to manage the code before they make it back into the main branches. In smaller projects, they are often unnecessary. Nonetheless it's good to know these branches as they will become useful in larger projects. 

Supporting branches are not limited to one per type - multiple supporting branches of the same type can run concurrently with different purposes for different teams.

#### Feature Branches

May branch off:

- *develop*

Must merge back into:

- *develop*

Branch naming convention:

- Anything except: 
  - *master*
  - *develop*
  - *release-\**
  - *hotfix-\**

Feature branches (or topic branches) are used to develop a specific new feature. The ultimate end of this branch is to be merged in as a new feature to *develop* or to be discarded if the feature is not required. Feature branches should take into account the state of the *develop* branch lest the feature become impossible to merge back when it is complete.

Feature branches should exist within a developer's computer only and not in the *origin*.

When creating a feature branch, disable *fast-forward* so as to not lose any historical information on the contribution of the feature branch to the *develop* branch. If GitKraken is used, the program will ask if you intend to fast forward during a merge or not.

The following allows you to visualize the loss in historical information:

![img](../../../Consistently%20Updated/coding-notes/Git/assets/merge-without-ff@2x.png)

#### Release Branches

May branch off:

- *develop*

Must merge back into:

- *develop* and *master*

Branch naming convention:

- *release-\**

The release branch is used to polish a *develop* state into a new release. In essence, it's use to dot the 'i's and cross the 't's of the code. This can be anything from removing debug messages to configuring the runtime parameters for a specific vendor. 

Only upon creation of a release branch does a version number get assigned to the release. This means that the *develop* branch never has a release number and is just in a perpetual state of development. The exact version number is up to version naming conventions or (in smaller projects) personal preference. As long as they as incremental and make a modicum of sense.

Here's an example of release branches for Arducopter - open-source drone firmware:

![1564661426621](../../../Consistently%20Updated/coding-notes/Git/assets/1564661426621.png)

After a *release* branch is completed, it is merged with the *master* branch and tagged with the appropriate version number for easy reference.

After that, the *release* branch has to be merged back to the *develop* branch as well to keep the changes made during that branch. Once that is done, the *release* branch is usually deleted.

#### Hotfix Branch

May branch off:

- *master*

Must merge back into:

- *develop* and *master*

Branch naming convention:

- *hotfix-\**

A *hotfix* branch is a smaller and more explosive version of a *release* branch. They arise directly from production code that requires stamping out of a bug or code amendment. This extra branch allows people working on the *develop* branch to continue uninterrupted while a quick response force deals with fixing a critical bug in the production code.

When completed, the *hotfix* branch is to be merged into the *master* and *develop* branches.

**However, if there is a *release* branch active, the *hotfix* branch is to be merged into the *release* branch instead of *develop* branch. The changes will be propagated to *develop* once the *release* branch is complete.**

## Commit Messages

Commit messages deserves a subsection of it's own because they are the first thing people (or even yourself) will read when trying to understand the commit. Writing good commit messages is also essential in trying to get your pull request approved as a poor commit message will leave the repository maintainer confused as to intent and purpose of the pull request.

### Some Examples

Here is an example of a few git commit messages. What do you notice?

![1564670491164](../../../Consistently%20Updated/coding-notes/Git/assets/1564670491164.png)

Everything is straight forward to understand and each commit's purpose is made clear.

Now here's another set of commit messages:

![1564670575453](../../../Consistently%20Updated/coding-notes/Git/assets/1564670575453.png)

What does "edit" even mean? What has been changed? Anyone interested would have to open up the source files to check what features may or may not have been added.

![1564670713444](../../../Consistently%20Updated/coding-notes/Git/assets/1564670713444.png)

Now this one is so long that even GitKraken could not fully show it.

### The Seven Rules of a Great Commit Message

The following have been taken from [Chris Beam's website](https://chris.beams.io/posts/git-commit/), though as he admits - this has all been said before time an again:

1. [Separate subject from body with a blank line](#rule1)
2. [Limit the subject line to 50 characters](#rule2)
3. [Capitalize the subject line](#rule3)
4. [Do not end the subject line with a period](#rule4)
5. [Use imperative mood in the subject line](#rule5)
6. [Wrap the body at 72 characters](#rule6)
7. [Use the body to explain *what* and *why* instead of *how*](#rule7)

#### 1. Separate subject from body with a blank line <a name="rule1"></a> 

As a rule of thumb, separate the subject of the commit with the body of the commit. This makes the commit messages much more readable whel called via `git log` or when browsing through the commit history on GitHub.

In GitKraken, the subject and body is divided into *Summary* and *Description* sections. A commit with both sections filled will look like this in the repository history on GitHub

![1564671383062](../../../Consistently%20Updated/coding-notes/Git/assets/1564671383062.png)

The bolded text above it the *Summary* while the words in smaller font below form the *Description*.

#### 2. Limit the subject line to 50 characters <a name="rule2"></a> 

Keeping the subject line at 50 characters makes the commit history more readable and also forces the author to think about brevity. If the commit is on GitHub, GitHub itself will warn if a commit message gets too long:

![1564671619233](../../../Consistently%20Updated/coding-notes/Git/assets/1564671619233.png)

#### 3. Capitalize the subject line <a name="rule3"></a> 

Simple enough, capitalize the subject line to make everything look better.

Example:

- Create docker image for version 1.4

Not:

- create docker image for version 1.4

#### 4. Do not end the subject line with a period <a name="rule4"></a> 

Another simple rule - additionally, using punctuation means using up 2% of the 50 character limit. 

Example:

- Add configuration generation script

Not:

- Add configuration generation script.

Additionally, if the commit message requires other punctuation like commas, semicolons and etc, it is probably too long.

#### 5. Use imperative mood in the subject line <a name="rule5"></a> 

Imperative mood means that that the messages should be written as if giving a command or instruction. This tends to make messages shorter and also follows the convention that Git uses imperative messages when doing commits on your behalf.

Examples:

- Add banana counting script
- Remove all global variables from utils.py

Not:

- Added banana counting script
- Removed all global variables from utils.py

If it helps, instead of thinking along the lines of "what have I added to the code?" (which is why people tend to write in past tense), think along the lines of "what will this change do to someone else's repository?"

- What I have done to the code: Added a banana counter (past tense)
- **What will this change do**: Add a banana counter (imperative mood)

Think along the latter, not the former.

*If you need more examples, the 7 rules have been written in imperative form*

#### 6. Wrap the body at 72 characters <a name="rule6"></a> 

This applies more for traditional CLI git commits. Because Git does not wrap text automatically, the lines will go on and on as far as the command line can stretch. Thus even in the body, remember to wrap the text once it hits 72 characters.

Once again this is not so applicable to GitKraken.

#### 7. Use the body to explain *what* and *why* instead of *how* <a name="rule7"></a> 

Commit messages should answer the question of what has been changed and why it has been changed. The question of how is self evident in the code. However, when maintaining large swaths of code, the usefulness of a commit is reliant on what it does and why it has to be done. 

This is used to saved time for everyone (including yourself in the future)

### Commit Messages in GitKraken

One great thing about using GitKraken is that it actively polices the 72 character limit:

![1564670879735](../../../Consistently%20Updated/coding-notes/Git/assets/1564670879735.png)

While it is possible to go over the limit, GitKraken will make it clear that the commit message is now not within optimal conventions. 

![1564670936504](../../../Consistently%20Updated/coding-notes/Git/assets/1564670936504.png)

It is still possible make the commit with the `-2` characters but you will not wallow in shame for disappointing the Git Gods.