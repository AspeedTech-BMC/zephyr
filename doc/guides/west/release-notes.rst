.. _west-release-notes:

West Release Notes
##################

v0.11.0
*******

New features:

- ``west update`` now supports ``--narrow``, ``--name-cache``, and
  ``--path-cache`` options. These can be influenced by the ``update.narrow``,
  ``update.name-cache``, and ``update.path-cache`` :ref:`west-config` options.
  These can be used to optimize the speed of the update.
- ``west update`` now supports a ``--fetch-opt`` option that will be passed to
  the ``git fetch`` command used to fetch remote revisions when updating each
  project.

Bug fixes:

- ``west update`` now synchronizes Git submodules in projects by default. This
  avoids issues if the URL changes in the manifest file from when the submodule
  was first initialized. This behavior can be disabled by setting the
  ``update.sync-submodules`` configuration option to ``false``.

Other changes:

- the :ref:`west-apis-manifest` module has fixed docstrings for the Project
  class

v0.10.1
*******

New features:

- The :ref:`west-init` command's ``--manifest-rev`` (``--mr``) option no longer
  defaults to ``master``. Instead, the command will query the repository for
  its default branch name and use that instead. This allows users to move from
  ``master`` to ``main`` without breaking scripts that do not provide this
  option.

v0.10.0
*******

New features:

- The ``name`` key in a project's :ref:`submodules list
  <west-manifest-submodules>` is now optional.

Bug fixes:

- West now checks that the manifest schema version is one of the explicitly
  allowed vlaues documented in :ref:`west-manifest-schema-version`. The old
  behavior was just to check that the schema version was newer than the west
  version where the ``manifest: version:`` key was introduced. This incorrectly
  allowed invalid schema versions, like ``0.8.2``.

Other changes:

- A manifest file's ``group-filter`` is now propagated through an ``import``.
  This is a change from how west v0.9.x handled this. In west v0.9.x, only the
  top level manifest file's ``group-filter`` had any effect; the group filter
  lists from any imported manifests were ignored.

  Starting with west v0.10.0, the group filter lists from imported manifests
  are also imported. For details, see :ref:`west-group-filter-imports`.

  The new behavior will take effect if ``manifest: version:`` is not given or
  is at least ``0.10``. The old behavior is still available in the top level
  manifest file only with an explicit ``manifest: version: 0.9``. See
  :ref:`west-manifest-schema-version` for more information on schema versions.

  See `west pull request #482
  <https://github.com/zephyrproject-rtos/west/pull/482>`_ for the motivation
  for this change and additional context.

v0.9.1
******

Bug fixes:

- Commands like ``west manifest --resolve`` now correctly include group and
  group filter information.

Other changes:

- West now warns if you combine ``import`` with ``group-filter``. Semantics for
  this combination have changed starting with v0.10.x. See the v0.10.0 release
  notes above for more information.

v0.9.0
******

.. warning::

   The ``west config`` fix described below comes at a cost: any comments or
   other manual edits in configuration files will be removed when setting a
   configuration option via that command or the ``west.configuration`` API.

.. warning::

   Combining the ``group-filter`` feature introduced in this release with
   manifest imports is discouraged. The resulting behavior has changed in west
   v0.10.

New features:

- West manifests now support :ref:`west-manifest-submodules`. This allows you
  to clone `Git submodules
  <https://git-scm.com/book/en/v2/Git-Tools-Submodules>`_ into a west project
  repository in addition to the project repository itself.

- West manifests now support :ref:`west-manifest-groups`. Project groups can be
  enabled and disabled to determine what projects are "active", and therefore
  will be acted upon by the following commands: ``west update``, ``west list``,
  ``west diff``, ``west status``, ``west forall``.

- ``west update`` no longer updates inactive projects by default. It now
  supports a ``--group-filter`` option which allows for one-time modifications
  to the set of enabled and disabled project groups.

- Running ``west list``, ``west diff``, ``west status``, or ``west forall``
  with no arguments does not print information for inactive projects by
  default. If the user specifies a list of projects explicitly at the command
  line, output for them is included regardless of whether they are active.

  These commands also now support ``--all`` arguments to include all
  projects, even inactive ones.

- ``west list`` now supports a ``{groups}`` format string key in its
  ``--format`` argument.

Bug fixes:

- The ``west config`` command and ``west.configuration`` API did not correctly
  store some configuration values, such as strings which contain commas. This
  has been fixed; see `commit 36f3f91e
  <https://github.com/zephyrproject-rtos/west/commit/36f3f91e270782fb05f6da13800f433a9c48f130>`_
  for details.

- A manifest file with an empty ``manifest: self: path:`` value is invalid, but
  west used to let it pass silently. West now rejects such manifests.

- A bug affecting the behavior of the ``west init -l .`` command was fixed; see
  `issue #435 <https://github.com/zephyrproject-rtos/west/issues/435>`_.

:ref:`API <west-apis>` changes:

- added ``west.manifest.Manifest.is_active()``
- added ``west.manifest.Manifest.group_filter``
- added ``submodules`` attribute to ``west.manifest.Project``, which has
  newly added type ``west.manifest.Submodule``

Other changes:

- The :ref:`west-manifest-import` feature now supports the terms ``allowlist``
  and ``blocklist`` instead of ``whitelist`` and ``blacklist``, respectively.

  The old terms are still supported for compatibility, but the documentation
  has been updated to use the new ones exclusively.

v0.8.0
******

This is a feature release which changes the manifest schema by adding support
for a ``path-prefix:`` key in an ``import:`` mapping, along with some other
features and fixes.

- Manifest import mappings now support a ``path-prefix:`` key, which places
  the project and its imported repositories in a subdirectory of the workspace.
  See :ref:`west-manifest-ex3.4` for an example.
- The west command line application can now also be run using ``python3 -m
  west``. This makes it easier to run west under a particular Python
  interpreter without modifying the :envvar:`PATH` environment variable.
- :ref:`west manifest --path <west-manifest-path>` prints the absolute path to
  west.yml
- ``west init`` now supports an ``--mf foo.yml`` option, which initializes the
  workspace using :file:`foo.yml` instead of :file:`west.yml`.
- ``west list`` now prints the manifest repository's path using the
  ``manifest.path`` :ref:`configuration option <west-config>`, which may differ
  from the ``self: path:`` value in the manifest data. The old behavior is
  still available, but requires passing a new ``--manifest-path-from-yaml``
  option.
- Various Python API changes; see :ref:`west-apis` for details.

v0.7.3
******

This is a bugfix release.

- Fix an error where a failed import could leave the workspace in an unusable
  state (see [PR #415](https://github.com/zephyrproject-rtos/west/pull/415) for
  details)

v0.7.2
******

This is a bugfix and minor feature release.

- Filter out duplicate extension commands brought in by manifest imports
- Fix ``west.Manifest.get_projects()`` when finding the manifest repository by
  path

v0.7.1
******

This is a bugfix and minor feature release.

- ``west update --stats`` now prints timing for operations which invoke a
  subprocess, time spent in west's Python process for each project, and total
  time updating each project.
- ``west topdir`` always prints a POSIX style path
- minor console output changes

v0.7.0
******

The main user-visible feature in west 0.7 is the :ref:`west-manifest-import`
feature. This allows users to load west manifest data from multiple different
files, resolving the results into a single logical manifest.

Additional user-visible changes:

- The idea of a "west installation" has been renamed to "west workspace" in
  this documentation and in the west API documentation. The new term seems to
  be easier for most people to work with than the old one.
- West manifests now support a :ref:`schema version
  <west-manifest-schema-version>`.
- The "west config" command can now be run outside of a workspace, e.g.
  to run ``west config --global section.key value`` to set a configuration
  option's value globally.
- There is a new :ref:`west topdir <west-built-in-misc>` command, which
  prints the root directory of the current west workspace.
- The ``west -vv init`` command now prints the git operations being performed,
  and their results.
- The restriction that no project can be named "manifest" is now enforced; the
  name "manifest" is reserved for the manifest repository, and is usable as
  such in commands like ``west list manifest``, instead of ``west list
  path-to-manifest-repository`` being the only way to say that
- It's no longer an error if there is no project named "zephyr". This is
  part of an effort to make west generally usable for non-Zephyr use cases.
- Various bug fixes.

The developer-visible changes to the :ref:`west-apis` are:

- west.build and west.cmake: deprecated; this is Zephyr-specific functionality
  and should never have been part of west. Since Zephyr v1.14 LTS relies on it,
  it will continue to be included in the distribution, but will be removed
  when that version of Zephyr is obsoleted.
- west.commands:

  - WestCommand.requires_installation: deprecated; use requires_workspace instead
  - WestCommand.requires_workspace: new
  - WestCommand.has_manifest: new
  - WestCommand.manifest: this is now settable
- west.configuration: callers can now identify the workspace directory
  when reading and writing configuration files
- west.log:

  - msg(): new
- west.manifest:

  - The module now uses the standard logging module instead of west.log
  - QUAL_REFS_WEST: new
  - SCHEMA_VERSION: new
  - Defaults: removed
  - Manifest.as_dict(): new
  - Manifest.as_frozen_yaml(): new
  - Manifest.as_yaml(): new
  - Manifest.from_file() and from_data(): these factory methods are more
    flexible to use and less reliant on global state
  - Manifest.validate(): new
  - ManifestImportFailed: new
  - ManifestProject: semi-deprecated and will likely be removed later.
  - Project: the constructor now takes a topdir argument
  - Project.format() and its callers are removed. Use f-strings instead.
  - Project.name_and_path: new
  - Project.remote_name: new
  - Project.sha() now captures stderr
  - Remote: removed

West now requires Python 3.6 or later. Additionally, some features may rely on
Python dictionaries being insertion-ordered; this is only an implementation
detail in CPython 3.6, but is is part of the language specification as of
Python 3.7.

v0.6.3
******

This point release fixes an error in the behavior of the deprecated
``west.cmake`` module.

v0.6.2
******

This point release fixes an error in the behavior of ``west
update --fetch=smart``, introduced in v0.6.1.

All v0.6.1 users must upgrade.

v0.6.1
******

.. warning::

   Do not use this point release. Make sure to use v0.6.2 instead.

The user-visible features in this point release are:

- The :ref:`west-update` command has a new ``--fetch``
  command line flag and ``update.fetch`` :ref:`configuration option
  <west-config>`. The default value, "smart", skips fetching SHAs and tags
  which are available locally.
- Better and more consistent error-handling in the ``west diff``, ``west
  status``, ``west forall``, and ``west update`` commands. Each of these
  commands can operate on multiple projects; if a subprocess related to one
  project fails, these commands now continue to operate on the rest of the
  projects. All of them also now report a nonzero error code from the west
  process if any of these subprocesses fails (this was previously not true of
  ``west forall`` in particular).
- The :ref:`west manifest <west-built-in-misc>` command also handles errors
  better.
- The :ref:`west list <west-built-in-misc>` command now works even when the
  projects are not cloned, as long as its format string only requires
  information which can be read from the manifest file. It still fails if the
  format string requires data stored in the project repository, e.g. if it
  includes the ``{sha}`` format string key.
- Commands and options which operate on git revisions now accept abbreviated
  SHAs. For example, ``west init --mr SHA_PREFIX`` now works. Previously, the
  ``--mr`` argument needed to be the entire 40 character SHA if it wasn't a
  branch or a tag.

The developer-visible changes to the :ref:`west-apis` are:

- west.log.banner(): new
- west.log.small_banner(): new
- west.manifest.Manifest.get_projects(): new
- west.manifest.Project.is_cloned(): new
- west.commands.WestCommand instances can now access the parsed
  Manifest object via a new self.manifest property during the
  do_run() call. If read, it returns the Manifest object or
  aborts the command if it could not be parsed.
- west.manifest.Project.git() now has a capture_stderr kwarg


v0.6.0
******

- No separate bootstrapper

  In west v0.5.x, the program was split into two components, a bootstrapper and
  a per-installation clone. See `Multiple Repository Management in the v1.14
  documentation`_ for more details.

  This is similar to how Google's Repo tool works, and lets west iterate quickly
  at first. It caused confusion, however, and west is now stable enough to be
  distributed entirely as one piece via PyPI.

  From v0.6.x onwards, all of the core west commands and helper classes are
  part of the west package distributed via PyPI. This eliminates complexity
  and makes it possible to import west modules from anywhere in the system,
  not just extension commands.
- The ``selfupdate`` command still exists for backwards compatibility, but
  now simply exits after printing an error message.
- Manifest syntax changes

  - A west manifest file's ``projects`` elements can now specify their fetch
    URLs directly, like so:

    .. code-block:: yaml

       manifest:
         projects:
           - name: example-project-name
             url: https://github.com/example/example-project

    Project elements with ``url`` attributes set in this way may not also have
    ``remote`` attributes.
  - Project names must be unique: this restriction is needed to support future
    work, but was not possible in west v0.5.x because distinct projects may
    have URLs with the same final pathname component, like so:

    .. code-block:: yaml

       manifest:
         remotes:
           - name: remote-1
             url-base: https://github.com/remote-1
           - name: remote-2
             url-base: https://github.com/remote-2
         projects:
           - name: project
             remote: remote-1
             path: remote-1-project
           - name: project
             remote: remote-2
             path: remote-2-project

    These manifests can now be written with projects that use ``url``
    instead of ``remote``, like so:

    .. code-block:: yaml

       manifest:
         projects:
           - name: remote-1-project
             url: https://github.com/remote-1/project
           - name: remote-2-project
             url: https://github.com/remote-2/project

- The ``west list`` command now supports a ``{sha}`` format string key

- The default format string for ``west list`` was changed to ``"{name:12}
  {path:28} {revision:40} {url}"``.

- The command ``west manifest --validate`` can now be run to load and validate
  the current manifest file, among other error-handling fixes related to
  manifest parsing.

- Incompatible API changes were made to west's APIs. Further changes are
  expected until API stability is declared in west v1.0.

  - The ``west.manifest.Project`` constructor's ``remote`` and ``defaults``
    positional arguments are now kwargs. A new ``url`` kwarg was also added; if
    given, the ``Project`` URL is set to that value, and the ``remote`` kwarg
    is ignored.

  - ``west.manifest.MANIFEST_SECTIONS`` was removed. There is only one section
    now, namely ``manifest``. The *sections* kwargs in the
    ``west.manifest.Manifest`` factory methods and constructor were also
    removed.

  - The ``west.manifest.SpecialProject`` class was removed. Use
    ``west.manifest.ManifestProject`` instead.


v0.5.x
******

West v0.5.x is the first version used widely by the Zephyr Project as part of
its v1.14 Long-Term Support (LTS) release. The `west v0.5.x documentation`_ is
available as part of the Zephyr's v1.14 documentation.

West's main features in v0.5.x are:

- Multiple repository management using Git repositories, including self-update
  of west itself
- Hierarchical configuration files
- Extension commands

Versions Before v0.5.x
**********************

Tags in the west repository before v0.5.x are prototypes which are of
historical interest only.

.. _Multiple Repository Management in the v1.14 documentation:
   https://docs.zephyrproject.org/1.14.0/guides/west/repo-tool.html

.. _west v0.5.x documentation:
   https://docs.zephyrproject.org/1.14.0/guides/west/index.html
