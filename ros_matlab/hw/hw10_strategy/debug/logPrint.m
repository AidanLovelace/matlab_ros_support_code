function logPrint(logLevel, logTopic, indentLevel, style, fmt, varargin)
    % logPrint  Conditionally prints a formatted log message with indentation and custom style.
    %   logLevel    - numeric severity level of this message
    %   logTopic    - char array labeling the topic (e.g. 'INFO', 'DEBUG')
    %   indentLevel - number of indent levels (each level == 3 spaces)
    %   style       - cprintf style string (e.g. 'text', 'Keywords'). Empty for default 'text'
    %   fmt         - format string, like sprintf
    %   varargin    - values matching the format specifier
    %
    % Requires a global variable `logVerbosity`. Messages whose logLevel is
    % greater than logVerbosity will not be printed.
    
    global logVerbosity;
    if isempty(logVerbosity)
        warning('logVerbosity is not set. Defaulting to 0.');
        logVerbosity = 0;
    end
    % Only print if level is within verbosity
    if logLevel > logVerbosity
        return;
    end
    % Determine style
    if isempty(style)
        style = 'text';
    end
    % Build indentation string
    indentStr = '';
    if indentLevel > 0
        totalSpaces = indentLevel * 3;
        indentStr = repmat(' ', 1, totalSpaces);
        % Replace the last 3 spaces with ' - ' (dash as bullet)
        indentStr(end-2:end) = ' - ';
    end
    % Get timestamp
    timestamp = char(datetime('now', 'Format', 'HH:mm:ss:SSS'));
    % Format the message body
    msgBody = sprintf(fmt, varargin{:});
    % Combine topic and message
    fullMsg = sprintf('[%s] %s[%s] %s', timestamp, indentStr, logTopic, msgBody);
    % Print using cprintf
    cprintf(style, '%s\n', fullMsg);
end