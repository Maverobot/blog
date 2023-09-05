+++
title = "Get started with sed"
author = ["Zheng Qu"]
date = 2020-06-26
lastmod = 2020-06-26T14:54:08+02:00
tags = ["sed"]
categories = ["cli"]
draft = false
weight = 2001
+++

## What is sed? {#what-is-sed}

`sed` is a "**s**tream **ed**itor for filtering and transforming text". Here is a very detailed [tutorial](https://www.grymoire.com/Unix/Sed.html).


## Basic usages {#basic-usages}


### Use "s" for substitution {#use-s-for-substitution}

Basic syntax:

```bash
echo "Sunday Monday" | sed 's/day/night/'
```

```text
Sunnight Monday
```

Note that only the first occurrence of `day` was replaced by `night`. To replace all, you have to use sed pattern flag `/g` for **g**lobal replacement.

```bash
echo "Sunday Monday" | sed 's/day/night/g'
```

```text
Sunnight Monnight
```


### Flexible delimiters {#flexible-delimiters}

I think we all agree that the following code is ugly:

```bash
echo "/usr/local/bin/n" | sed 's/\/usr\/local/\/usr/'
```

With `sed`, the delimiter does not need to be `/`. It can be `/`, `_`,  `-`, `#`, `*` etc. Pick the one fit you the best.

```bash
echo "/usr/local/bin/n" | sed 's_/usr/local_/usr_'
echo "/usr/local/bin/n" | sed 's-/usr/local-/usr-'
echo "/usr/local/bin/n" | sed 's#/usr/local#/usr#'
echo "/usr/local/bin/n" | sed 's*/usr/local*/usr*'
echo "/usr/local/bin/n" | sed 's</usr/local</usr<'
# ...
```

[//]: # "Exported with love from a post written in Org mode"
[//]: # "- https://github.com/kaushalmodi/ox-hugo"
