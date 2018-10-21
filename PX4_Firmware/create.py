import sys, datetime, os, os.path

# helper for showing help information
def ShowHelp():
    print 'create.py - A helper for generating C++ code files.'
    print '  Usage: python create.py [class/file name] [options]'
    print '  Options:'
    print '    --creator [name]     - Specifies the copyright holder for the file headers.'
    print '    --directory [dir]    - The directory in which to create the files.'
    print '    --namespace [name]   - Specifies the namespace of the type. Can be nested, i.e. Foo::Bar.'
    print '  Examples:'
    print '    python create.py MyClass'
    print '    python create.py SomeClass --namespace SomeNamespace'
    print '    python create.py Sprite --creator "Nick Gravelyn" --namespace "MyGame::Graphics"'

# helper for writing indented text
def WriteIndent(theFile, indentCount, text):
    while (indentCount > 0):
        text = '    ' + text
        indentCount -= 1
    theFile.write(text)

# helper for writing the copyright notice on top of the files
def WriteCopyright(theFile, filename, creator):
    theFile.write('//\n//  ' + filename + '\n//  Copyright ' + str(datetime.datetime.now().year))
    if (creator != ''):
        theFile.write(' ' + creator)
    theFile.write('. All rights reserved.\n//\n\n')

# checks for an option in the arguments. returns false if the option isn't given.
def GetOption(option):
    if (option in sys.argv):
        index = sys.argv.index(option)
        if (len(sys.argv) < index + 1):
            print 'ERROR: Missing value for ' + option
            ShowHelp()
            sys.exit()
        return sys.argv[index + 1]
    else:
        return False

# we need at least the name
if (len(sys.argv) < 2):
    ShowHelp()
    sys.exit()

# get the item name
itemName = sys.argv[1]

# get the creator's name
creator = GetOption('--creator')
if (creator == False):
    creator = ''

# get the directory for the files
directory = GetOption('--directory')
if (directory == False):
    directory = '.'

# get the namespace list
namespaces = GetOption('--namespace')
if (namespaces == False):
    namespaces = []
else:
    namespaces = namespaces.split('::')

# get our file paths
headerPath = os.path.join(directory, itemName + '.hpp')
implPath = os.path.join(directory, itemName + '.cpp')

# create our header file first
defineGuard = itemName.upper() + '_HPP_INCLUDED'
print 'Creating ' + headerPath
with open(headerPath, 'w') as headerFile:
    # write the copyright header
    WriteCopyright(headerFile, itemName + '.hpp', creator)

    # write the include guards
    headerFile.write('#ifndef ' + defineGuard + '\n#define ' + defineGuard + '\n\n')
    
    # write the namespaces
    t = 0
    for n in namespaces:
        WriteIndent(headerFile, t, 'namespace ' + n + '\n')
        WriteIndent(headerFile, t, '{\n')
        t = t + 1

    # write the basic class definition
    WriteIndent(headerFile, t, 'class ' + itemName + '\n')
    WriteIndent(headerFile, t, '{\n')
    WriteIndent(headerFile, t, 'public:\n')
    WriteIndent(headerFile, t + 1, itemName + '();\n')
    WriteIndent(headerFile, t + 1, 'virtual ~' + itemName + '();\n')
    WriteIndent(headerFile, t, '};\n')

    # write the close brackets for the namespaces
    for n in namespaces:
        t = t - 1
        WriteIndent(headerFile, t, '}\n')

    # wrie the end for the include guard
    headerFile.write('\n#endif // ' + defineGuard + '\n\n')

# now write out the implementation file
print 'Creating ' + implPath
with open(implPath, 'w') as implFile:
    # write the copyright header
    WriteCopyright(implFile, itemName + '.cpp', creator)

    # add the include for the header file
    implFile.write('#include "' + itemName + '.hpp"\n\n')

    # add the using statement
    if (len(namespaces) > 0):
        implFile.write('using namespace ' + '::'.join(namespaces) + '\n\n')

    # write the implementation sections for constructor/destructor
    implFile.write(itemName + '::' + itemName + '()\n{\n}\n\n')
    implFile.write(itemName + '::~' + itemName + '()\n{\n}\n\n')
