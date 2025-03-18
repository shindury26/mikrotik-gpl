# mikrotik-gpl

> [!WARNING]
> This project is a work in progress.  At time of writing, not GPL source code can be shared by MikroTik after request.

Mikrotik does not publish GPL source code, specifically the Linux kernel, on its website or other downloadable place.  Instead, it requires a email request to obtain the source code.  So this repo was created to automate the process and provide it for direct download from GitHub when source is provided by Mikrotik.

> [!TIP]
> Source code from RouterOS 6.41rc38 was found on this unrelated GitHub project, from 8 years ago:
> https://github.com/robimarko/routeros-GPL


### Implementation Details

When the GPL/open source code is, eventually, obtained from MikroTik, it will be manually checked into repo.    This project is mainly store those GPL disclousre publicly.  So the only "code" here is a GitHub Action, `gpl-nag-email.yaml`.  The build script checks MikroTik website for the current "stable" and "latest" versions of RouterOS.  If a version is NOT found in GitHub [Releases](https://github.com/tikoci/mikrotik-gpl/releases), the build will email MikroTik to obtain the GPL source.  The build is schedule to check for new releases daily, and will not send emails if a version has already been requested.

### Potential Uses

> [!NOTE]
> Significant parts of RouterOS are NOT GPL or other open source code, and would not require disclosure. 
> RouterOS does use the Linux kernel, so that is main GPL-based code, and will be posted here when obtained from MikroTik.  But the kernel alone is not enough to "rebuild" RouterOS or "add custom packages", as most of Linux "userland" is propritary code and/or may use non-public 3rd party drivers.  So the code here may not be useful, depending on your goals.

GPL and other open source licenses do not require a reason to want disclosure.  

*  a `diff` could be used similar to [tikoci/restraml's "Scheme Tools"](https://tikoci.github.io/restraml) that use  `/console/inspect`, and similarly enable better visibility into any kernel changes – providing another view into the opaque releases notes
* Run GitHub basis static code analysis tools against disclosed GPL code - even if not great, another datapoint on a new release 
* Perhaps useful in various "re-packaging" CHR/X86 project in TIKOCI - perhaps improving EUFI and knowing kernel options used to better align virtualization settings