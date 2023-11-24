boolean areStringsEqual(const String str1, const String str2) {
  // Convert String objects to char arrays for strcmp
  char charArray1[str1.length() + 1];
  char charArray2[str2.length() + 1];
  str1.toCharArray(charArray1, sizeof(charArray1));
  str2.toCharArray(charArray2, sizeof(charArray2));

  // Compare strings
  return strcmp(charArray1, charArray2) == 0;
}

String replaceFirstOccurrence(const String input, const String& match, const String& replace) {
  String output = input;
  int startPos = output.indexOf(match);

  if (startPos != -1) {
    output = output.substring(0, startPos) + replace + output.substring(startPos + match.length());
  }
  return output;
}

String camelCaseToWordsUntillFirstColon(String input) {
  String output = "";
  bool colonFound = false;

  for (int i = 0; i < input.length(); i++) {
    if (input[i] == ':') {
      colonFound = true;
    }

    if (!colonFound) {
      if (i > 0 && isUpperCase(input[i]) && !isUpperCase(input[i - 1])) {
        output += " ";  // Add a space before adding the uppercase letter
        output += input[i];
      } else {
        output += input[i];
      }
    } else {
      output += input[i];
    }
  }

  return output;
}


/*
String replaceString(String input, const String& search, const String& replace) {
  int index = 0;
  while ((index = input.indexOf(search, index)) != -1) {
    input = input.substring(0, index) + replace + input.substring(index + search.length());
    index += replace.length();
  }
  return input;
}
*/

String convertToUppercaseBeforeColon(String input) {
  int colonIndex = input.indexOf(':');  // Find the position of the first colon

  if (colonIndex != -1) {
    // Extract the part before the colon
    String partBeforeColon = input.substring(0, colonIndex);

    // Convert the extracted part to uppercase
    partBeforeColon.toUpperCase();

    // Construct the final output string
    String output = partBeforeColon + input.substring(colonIndex);

    return output;
  } else {
    // If no colon is found, return the input string as it is
    return input;
  }
}

String replaceMultipleSpaces(String input) {
  while (input.indexOf("  ") != -1) {
    input.replace("  ", " ");
  }
  return input;
}

String modifyStringToCapitalAfterColon(String input) {
  // Find the position of ": "
  int colonSpaceIndex = input.indexOf(": ");

  // Check if ": " was found
  if (colonSpaceIndex != -1 && colonSpaceIndex < input.length() - 2) {
    // Get the character after ": "
    char charToCapitalize = input.charAt(colonSpaceIndex + 2);

    // Check if the character is an alphabet letter
    if (isAlpha(charToCapitalize)) {
      // Convert the character to uppercase
      charToCapitalize = toupper(charToCapitalize);

      // Replace the original character with the uppercase one
      input.setCharAt(colonSpaceIndex + 2, charToCapitalize);
    }
  }

  // Return the modified string
  return input;
}

String removeLastFullStop(String inputString) {
  int stringLength = inputString.length();

  // Check if the string is empty or if the last character is not a period
  if (stringLength == 0 || inputString.charAt(stringLength - 1) != '.') {
    return inputString;
  }

  // Remove the last character (period)
  inputString.remove(stringLength - 1);
  return inputString;
}

String trimString(String inputString) {
  // Trim leading spaces
  int startIndex = 0;
  while (inputString.charAt(startIndex) == ' ') {
    startIndex++;
  }

  // Trim trailing spaces
  int endIndex = inputString.length() - 1;
  while (inputString.charAt(endIndex) == ' ') {
    endIndex--;
  }

  // Extract the trimmed substring
  String trimmedString = inputString.substring(startIndex, endIndex + 1);

  return trimmedString;
}