//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "CoreUtils/Log.h"

#include <filesystem>
#include <iostream> // std::cout
#include <regex>
#include <string>   // std::string::operator<<
#include <vector>

#include <Containers/UnrealString.h> // FString::operator*
#include <HAL/Platform.h>            // TEXT
#include <Logging/LogMacros.h>       // DECLARE_LOG_CATEGORY_EXTERN, DEFINE_LOG_CATEGORY, UE_LOG

#include "CoreUtils/Unreal.h"
#include "CoreUtils/Std.h"

DECLARE_LOG_CATEGORY_EXTERN(LogSpear, Log, All);
DEFINE_LOG_CATEGORY(LogSpear);

void Log::logCurrentFunction(const std::filesystem::path& current_file, const std::string& current_function)
{
    log(current_file, getCurrentFunctionAbbreviated(current_function));
}

void Log::logStdout(const std::string& str)
{
    std::cout << str << std::endl;
}

void Log::logUnreal(const std::string& str)
{
    // We need to use TEXT() instead of *Unreal::toFString() because the * operator doesn't return a const pointer
    UE_LOG(LogSpear, Log, TEXT("%s"), *Unreal::toFString(str));
}

std::string Log::getPrefix(const std::filesystem::path& current_file)
{
    return "[SPEAR | " + getCurrentFileAbbreviated(current_file) + "] ";
}

std::string Log::getCurrentFileAbbreviated(const std::filesystem::path& current_file)
{
    return current_file.filename().string();
}

std::string Log::getCurrentFunctionAbbreviated(const std::string& current_function)
{
    // This function expects an input string in the format used by the BOOST_CURRENT_FUNCTION macro, which can vary depending on the compiler.
    //
    // MSVC:
    //     __cdecl MyClass::MyClass(const class MyInputType1 &, const class MyInputType2 &, ...)
    //     MyReturnType __cdecl MyClass::myFunction<MyReturnType>(const class MyInputType1 &, const class MyInputType2 &, ...)
    //
    // Clang:
    //     MyClass::MyClass(const MyInputType1 &, const MyInputType2 &, ...)
    //     virtual MyReturnType MyClass::myFunction()
    //
    // Due to this variability, the most robust strategy for obtaining a sensible abbreviated function name seems to be the following: replace
    // all template expressions and function arguments with simplified strings, then tokenize, then return the token that contains "(" and ")".
    
    // Make a copy of the input string so we can simplify it in-place.
    std::string current_function_simplified = current_function;

    // Iteratively simplify template expressions with "<...>". We do this iteratively, because regular expressions are not intended to handle
    // arbitrarily nested brackets.
    std::regex template_expression_regex("<(([a-zA-Z_:*&, ])|(<\\.\\.\\.>))+>");
    while (std::regex_search(current_function_simplified, template_expression_regex)) {
        current_function_simplified = std::regex_replace(current_function_simplified, template_expression_regex, "<...>");
    }

    // Simplify function arguments, either with "()" or "(...)".
    std::regex function_void_arguments_regex("\\(void\\)");
    std::regex function_non_void_arguments_regex("\\((([a-zA-Z_:*&, ])|(<\\.\\.\\.>))+\\)");
    current_function_simplified = std::regex_replace(current_function_simplified, function_void_arguments_regex, "()");
    current_function_simplified = std::regex_replace(current_function_simplified, function_non_void_arguments_regex, "(...)");

    // Return the token containing "(" and ")".
    for (auto& token : Std::tokenize(current_function_simplified, " ")) {
        if (Std::containsSubstring(token, "(") && Std::containsSubstring(token, ")")) {
            return token;
        }
    }

    SP_ASSERT(false);
    return "";
}
